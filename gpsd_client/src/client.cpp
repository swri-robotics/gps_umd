#include <rclcpp/rclcpp.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps.h>

#include <gpsd_client/gpsd_parser_factory.hpp>
#include <gpsd_client/gpsd_raw_message.hpp>

#include <chrono>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

using namespace std::chrono_literals;

namespace gpsd_client
{
  class GPSDClientComponent : public rclcpp::Node
  {
  public:
    explicit GPSDClientComponent(const rclcpp::NodeOptions& options) :
      Node("gpsd_client", options),
      use_gps_time_(true),
      check_fix_by_variance_(false),
      override_augmentation_source_(false),
      publish_gpsd_raw_(false),
      publish_gpsd_rtcm_(false),
      frame_id_("gps"),
      publish_rate_(10)
    {
      if (!start()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start gpsd_client; timer not created.");
        return;
      }
      timer_ = create_wall_timer(publish_period_ms, std::bind(&GPSDClientComponent::step, this));
      RCLCPP_INFO(this->get_logger(), "Instantiated.");
    }

    /* gpsmm used to close the connection for us. Reading through the C API
     * means owning that: the socket and libgps's private buffer are leaked
     * otherwise, which matters because components are loaded and unloaded
     * within a running container.
     */
    ~GPSDClientComponent() override
    {
      timer_.reset();
      if (gps_opened_)
      {
        gps_stream(&gps_data_, WATCH_DISABLE, nullptr);
        gps_close(&gps_data_);
        gps_opened_ = false;
      }
    }

    bool start()
    {
      this->declare_parameter("use_gps_time", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("check_fix_by_variance", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("override_augmentation_source", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("publish_gpsd_raw", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("publish_gpsd_rtcm", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
      this->declare_parameter("publish_rate", rclcpp::PARAMETER_INTEGER);
      this->declare_parameter("host", rclcpp::PARAMETER_STRING);
      this->declare_parameter("port", rclcpp::PARAMETER_INTEGER);

      gps_fix_pub_ = create_publisher<gps_msgs::msg::GPSFix>("extended_fix", 1);
      navsatfix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);

      this->get_parameter_or("use_gps_time", use_gps_time_, use_gps_time_);
      this->get_parameter_or("check_fix_by_variance", check_fix_by_variance_, check_fix_by_variance_);
      this->get_parameter_or("override_augmentation_source", override_augmentation_source_,
                             override_augmentation_source_);
      this->get_parameter_or("publish_gpsd_raw", publish_gpsd_raw_, publish_gpsd_raw_);
      this->get_parameter_or("publish_gpsd_rtcm", publish_gpsd_rtcm_,
                             publish_gpsd_rtcm_);
      this->get_parameter_or("frame_id", frame_id_, frame_id_);
      this->get_parameter_or("publish_rate", publish_rate_, publish_rate_);

      if (publish_rate_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid publish_rate %d; using 1 Hz", publish_rate_);
        publish_rate_ = 1;
      }

      publish_period_ms = std::chrono::milliseconds{(int)(1000 / publish_rate_)};

      ParserContext context{frame_id_, use_gps_time_, check_fix_by_variance_,
                            override_augmentation_source_};
      parser_ = GpsdParserFactory::create(context);

      /* The raw topic is opt-in, and both the publisher and the parser are
       * created only when it is enabled. A full gps_data_t is far larger than
       * a GPSFix -- the skyview alone can run to a couple of hundred
       * satellites -- so nothing is serialized or advertised for the vast
       * majority of users who never ask for it.
       */
      /* The raw parser serves both topics, so it is created when either is
       * enabled. Neither topic is advertised unless asked for.
       */
      if (publish_gpsd_raw_ || publish_gpsd_rtcm_)
      {
        raw_parser_ = GpsdParserFactory::createRaw(context);
      }

      if (publish_gpsd_rtcm_)
      {
        /* RTCM is published apart from the raw report: the two RTCM families
         * are roughly half of all the generated message types, and a consumer
         * of corrections is rarely the same one that wants a fix. GPSDRaw
         * still carries SET_RTCM2/SET_RTCM3 in its `set` mask, so a raw
         * subscriber can tell an RTCM message arrived and look here for it.
         */
        gpsd_rtcm2_pub_ = create_publisher<GpsdRtcm2Msg>("gpsd_rtcm2", 1);
        gpsd_rtcm3_pub_ = create_publisher<GpsdRtcm3Msg>("gpsd_rtcm3", 1);
        RCLCPP_INFO(this->get_logger(),
                    "Publishing RTCM on gpsd_rtcm2 and gpsd_rtcm3");
      }

      if (publish_gpsd_raw_)
      {
        gpsd_raw_pub_ = create_publisher<GpsdRawMsg>("gpsd_raw", 1);
        RCLCPP_INFO(this->get_logger(),
                    "Publishing raw gpsd reports on gpsd_raw as %s "
                    "(libgps API %d.%d)",
                    GPSD_RAW_MESSAGE_NAME, GPSD_API_MAJOR_VERSION,
                    GPSD_API_MINOR_VERSION);
      }

      /* These must be members, not locals. gps_open() stores the host and
       * port pointers verbatim in gps_data_t::source (libgps_core.c) and never
       * copies them, so passing a local's c_str() leaves gpsd's own view of
       * where the data came from pointing at freed stack memory as soon as
       * this function returns.
       *
       * libgps does not read them back, so this was dormant -- but source is
       * part of every report handed to the parsers, and gpsd's own clients do
       * read source.server/port, so anything reaching for them would have been
       * undefined behaviour. Owning the strings for the node's lifetime costs
       * nothing and removes the trap.
       *
       * Neither may be reassigned after the connection is opened: that would
       * reallocate and dangle the pointers again.
       */
      host_ = "localhost";
      int port = atoi(DEFAULT_GPSD_PORT);
      this->get_parameter_or("host", host_, host_);
      this->get_parameter_or("port", port, port);
      port_ = std::to_string(port);

      if (0 != gps_open(host_.c_str(), port_.c_str(), &gps_data_))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPSd");
        return false;
      }
      gps_opened_ = true;

      if (-1 == gps_stream(&gps_data_, WATCH_ENABLE, nullptr))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to start the GPSd stream");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "GPSd opened");
      return true;
    }

    /* The JSON class of the report gps_read() just parsed, empty if there is
     * none.
     *
     * A view into the caller's buffer rather than a copy: it is only ever
     * compared, and it is consumed before the next read overwrites the buffer.
     * That also means there is no length limit to get wrong -- an earlier
     * version copied into a fixed member array and had to decide what to do
     * with a class name too long to fit.
     */
    static std::string_view reportClass(const char * message)
    {
      if (nullptr == message)
      {
        return {};
      }
      const char * key = std::strstr(message, "\"class\":\"");
      if (nullptr == key)
      {
        return {};
      }
      key += sizeof("\"class\":\"") - 1;
      const char * end = std::strchr(key, '"');
      if (nullptr == end)
      {
        return {};
      }
      return std::string_view(key, static_cast<std::size_t>(end - key));
    }

    /* RTCM is published per report, not per publish cycle.
     *
     * Everything else on this node is a state topic: a fix has a current
     * value, and publishing the latest one each cycle is the right thing.
     * Differential corrections are not that. They are a stream of discrete
     * messages, each meaningful once, so sampling them at publish_rate is
     * wrong in both directions:
     *
     *   Duplicates. libgps never resets gps_data_t::set -- only a successful
     *   parse assigns it, in gps_unpack()'s per-class handlers. A read that
     *   yields no new report therefore leaves both the mask and the union arm
     *   holding the *previous* report, and publishing on a timer republishes
     *   it. Measured against gpsd's own ublox-zed-f9r log: 44 RTCM3 messages
     *   published for 7 distinct payloads.
     *
     *   Loss. When several reports arrive within one cycle the drain keeps
     *   only the last, so every RTCM report but that one is discarded --
     *   including by the arm being overwritten, not merely unpublished.
     *
     * Both are fixed by draining report by report and publishing only when
     * the report just parsed *is* the RTCM one -- which the mask cannot tell
     * us. The mask is not per-report for every class: gpsd's TPV handler
     * assigns `set` outright and so clears RTCM3_SET, but its SKY handler
     * only ORs SATELLITE_SET in and never touches UNION_SET. So RTCM3_SET,
     * and the union arm behind it, survive every SKY report until some later
     * report happens to clear them. Measured on ublox-zed-f9r, where SKY
     * outnumbers TPV nine to one: 20 of 51 messages carrying RTCM3_SET also
     * carried SATELLITE_SET, a combination no genuine RTCM3 report produces.
     *
     * The report's own JSON class is the only reliable discriminator, and
     * gps_read() hands it back in its message argument. That is why this node
     * reads through the C API rather than gpsmm::read(), which passes NULL
     * there and throws the line away.
     *
     * Deliberately left alone: the fix topics still publish once per cycle at
     * publish_rate, which is what that parameter has always meant.
     */
    void publishRtcm(const gps_data_t & data, std::string_view report_class)
    {
      if (!gpsd_rtcm2_pub_ ||
          ("RTCM2" != report_class && "RTCM3" != report_class))
      {
        return;
      }

      rclcpp::Time now = this->get_clock()->now();

      /* Each parse returns nullopt unless this report actually is one --
       * gps_data_t packs the report arms into a union, so the set mask is
       * what makes reading the arm defined at all. The class check above has
       * already established which one this is; the mask check inside the
       * parser is what makes reading the arm defined.
       */
      std::optional<GpsdRtcm2Msg> rtcm2 = raw_parser_->parseRtcm2(data, now);
      if (rtcm2.has_value())
      {
        RCLCPP_DEBUG(this->get_logger(), "Publishing RTCM2...");
        gpsd_rtcm2_pub_->publish(*rtcm2);
      }

      std::optional<GpsdRtcm3Msg> rtcm3 = raw_parser_->parseRtcm3(data, now);
      if (rtcm3.has_value())
      {
        RCLCPP_DEBUG(this->get_logger(), "Publishing RTCM3...");
        gpsd_rtcm3_pub_->publish(*rtcm3);
      }
    }

    void step()
    {
      if (!gps_waiting(&gps_data_, 1000000))
        return;

      /* Read out all queued data and only act on the latest -- except for
       * RTCM, which is drained report by report. See publishRtcm().
       */
      /* Not a pointer: gps_read() fills our own gps_data_, so the only
       * question left is whether this cycle parsed anything at all.
       */
      bool have_report = false;
      while (gps_waiting(&gps_data_, 0))
      {
        message_[0] = '\0';
        if (0 >= gps_read(&gps_data_, message_, static_cast<int>(sizeof(message_))))
        {
          break;    // read error, or the connection closed
        }
        have_report = true;
        publishRtcm(gps_data_, reportClass(message_));
      }

      if (!have_report || !parser_->isOnline(gps_data_))
        return;

      rclcpp::Time now = this->get_clock()->now();

      RCLCPP_DEBUG(this->get_logger(), "Publishing gps fix...");
      gps_fix_pub_->publish(parser_->parseGpsFix(gps_data_, now));

      /* Published from the same report as the other two, so a subscriber can
       * line them up by timestamp. Deliberately not gated on
       * check_fix_by_variance: that filter exists to hide gpsd's stale-fix
       * behaviour from consumers of NavSatFix, and suppressing a report here
       * would make the "raw" topic a filtered one.
       */
      if (gpsd_raw_pub_)
      {
        RCLCPP_DEBUG(this->get_logger(), "Publishing raw gpsd report...");
        gpsd_raw_pub_->publish(raw_parser_->parseRaw(gps_data_, now));
      }

      std::optional<sensor_msgs::msg::NavSatFix> navsat_fix = parser_->parseNavSatFix(gps_data_, now);
      if (navsat_fix.has_value())
      {
        RCLCPP_DEBUG(this->get_logger(), "Publishing navsatfix...");
        navsatfix_pub_->publish(*navsat_fix);
      }
      else
      {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(),
          *this->get_clock(),
          1000,
          "GPS status was reported as OK, but variance was invalid");
      }
    }

  private:
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;
    /// Null unless publish_gpsd_raw is set; doubles as the enabled flag.
    rclcpp::Publisher<GpsdRawMsg>::SharedPtr gpsd_raw_pub_;
    /// Null unless publish_gpsd_rtcm is set; both share that one flag.
    rclcpp::Publisher<GpsdRtcm2Msg>::SharedPtr gpsd_rtcm2_pub_;
    rclcpp::Publisher<GpsdRtcm3Msg>::SharedPtr gpsd_rtcm3_pub_;

    /* Declared before gps_data_ on purpose. Members are destroyed in reverse
     * declaration order, so these outlive the connection that holds pointers
     * into them. See the note in start().
     */
    std::string host_;
    std::string port_;

    struct gps_data_t gps_data_ {};
    bool gps_opened_{false};

    /* Scratch for the raw JSON line gps_read() copies back, sized to libgps's
     * own buffer rather than to what a report "should" need.
     *
     * That is not defensiveness, it is required. gpsd 3.24 and newer overwrite
     * the caller's message_len with the actual line length before copying:
     *
     *     message_len = 1 + eol - PRIVATE(gpsdata)->buffer;
     *     memcpy(message, PRIVATE(gpsdata)->buffer, message_len);
     *
     * so the size passed in is ignored and a short buffer is overrun. Older
     * releases bound it properly with strlcpy, which is exactly the sort of
     * within-a-version-range difference that makes "it worked on 3.20" no
     * evidence at all. Sizing to the internal buffer is safe on both.
     */
#ifdef GPS_JSON_RESPONSE_MAX
    char message_[GPS_JSON_RESPONSE_MAX * 2] {};
#else
    // Not defined before API 14; this is the value those releases used.
    char message_[10240 * 2] {};
#endif

    std::unique_ptr<GpsdParser> parser_;
    std::unique_ptr<GpsdRawParser> raw_parser_;

    bool use_gps_time_;
    bool check_fix_by_variance_;
    bool override_augmentation_source_;
    bool publish_gpsd_raw_;
    bool publish_gpsd_rtcm_;
    std::string frame_id_;
    int publish_rate_;
    std::chrono::milliseconds publish_period_ms{};
    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GPSDClientComponent)
