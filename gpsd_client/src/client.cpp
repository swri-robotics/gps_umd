#include <rclcpp/rclcpp.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <libgpsmm.h>

#include <gpsd_client/gpsd_parser_factory.hpp>
#include <gpsd_client/gpsd_raw_message.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace gpsd_client
{
  class GPSDClientComponent : public rclcpp::Node
  {
  public:
    explicit GPSDClientComponent(const rclcpp::NodeOptions& options) :
      Node("gpsd_client", options),
      gps_(nullptr),
      use_gps_time_(true),
      check_fix_by_variance_(false),
      override_augmentation_source_(false),
      publish_gpsd_raw_(false),
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

    bool start()
    {
      this->declare_parameter("use_gps_time", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("check_fix_by_variance", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("override_augmentation_source", rclcpp::PARAMETER_BOOL);
      this->declare_parameter("publish_gpsd_raw", rclcpp::PARAMETER_BOOL);
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
      if (publish_gpsd_raw_)
      {
        raw_parser_ = GpsdParserFactory::createRaw(context);
        gpsd_raw_pub_ = create_publisher<GpsdRawMsg>("gpsd_raw", 1);
        RCLCPP_INFO(this->get_logger(),
                    "Publishing raw gpsd reports on ~/gpsd_raw as %s "
                    "(libgps API %d.%d)",
                    GPSD_RAW_MESSAGE_NAME, GPSD_API_MAJOR_VERSION,
                    GPSD_API_MINOR_VERSION);
      }

      std::string host = "localhost";
      int port = atoi(DEFAULT_GPSD_PORT);
      this->get_parameter_or("host", host, host);
      this->get_parameter_or("port", port, port);

      char port_s[12];
      snprintf(port_s, sizeof(port_s), "%d", port);

      gps_ = std::make_unique<gpsmm>(host.c_str(), port_s);
      if (gps_->stream(WATCH_ENABLE) == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPSd");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "GPSd opened");
      return true;
    }

    void step()
    {
      if (!gps_->waiting(1e6))
        return;

      // Read out all queued data and only act on the latest
      gps_data_t* p = nullptr;
      while (gps_->waiting(0))
      {
        p = gps_->read();
      }

      if (p == nullptr || !parser_->isOnline(*p))
        return;

      rclcpp::Time now = this->get_clock()->now();

      RCLCPP_DEBUG(this->get_logger(), "Publishing gps fix...");
      gps_fix_pub_->publish(parser_->parseGpsFix(*p, now));

      /* Published from the same report as the other two, so a subscriber can
       * line them up by timestamp. Deliberately not gated on
       * check_fix_by_variance: that filter exists to hide gpsd's stale-fix
       * behaviour from consumers of NavSatFix, and suppressing a report here
       * would make the "raw" topic a filtered one.
       */
      if (gpsd_raw_pub_)
      {
        RCLCPP_DEBUG(this->get_logger(), "Publishing raw gpsd report...");
        gpsd_raw_pub_->publish(raw_parser_->parseRaw(*p, now));
      }

      std::optional<sensor_msgs::msg::NavSatFix> navsat_fix = parser_->parseNavSatFix(*p, now);
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

    std::unique_ptr<gpsmm> gps_;
    std::unique_ptr<GpsdParser> parser_;
    std::unique_ptr<GpsdRawParser> raw_parser_;

    bool use_gps_time_;
    bool check_fix_by_variance_;
    bool override_augmentation_source_;
    bool publish_gpsd_raw_;
    std::string frame_id_;
    int publish_rate_;
    std::chrono::milliseconds publish_period_ms{};
    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GPSDClientComponent)
