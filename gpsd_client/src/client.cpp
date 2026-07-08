#include <rclcpp/rclcpp.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <gps_msgs/msg/gps_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <libgpsmm.h>

#include <chrono>
#include <cmath>

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
      check_fix_by_variance_(true),
      frame_id_("gps"),
      publish_rate_(1)
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
      this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
      this->declare_parameter("publish_rate", rclcpp::PARAMETER_INTEGER);
      this->declare_parameter("host", rclcpp::PARAMETER_STRING);
      this->declare_parameter("port", rclcpp::PARAMETER_INTEGER);

      gps_fix_pub_ = create_publisher<gps_msgs::msg::GPSFix>("extended_fix", 1);
      navsatfix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);

      this->get_parameter_or("use_gps_time", use_gps_time_, use_gps_time_);
      this->get_parameter_or("check_fix_by_variance", check_fix_by_variance_, check_fix_by_variance_);
      this->get_parameter_or("frame_id", frame_id_, frame_id_);
      this->get_parameter_or("publish_rate", publish_rate_, publish_rate_);

      if (publish_rate_ <= 0) {
        RCLCPP_WARN(this->get_logger(), "Invalid publish_rate %d; using 1 Hz", publish_rate_);
        publish_rate_ = 1;
      }

      publish_period_ms = std::chrono::milliseconds{(int)(1000 / publish_rate_)};

      std::string host = "localhost";
      int port = atoi(DEFAULT_GPSD_PORT);
      this->get_parameter_or("host", host, host);
      this->get_parameter_or("port", port, port);

      char port_s[12];
      snprintf(port_s, sizeof(port_s), "%d", port);

      gps_data_t* resp = nullptr;

#if GPSD_API_MAJOR_VERSION >= 5
      gps_ = new gpsmm(host.c_str(), port_s);
      resp = gps_->stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 4
      gps = new gpsmm();
      gps->open(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#else
      gps = new gpsmm();
      resp = gps->open(host.c_str(), port_s);
      gps->query("w\n");
#endif

      if (resp == nullptr)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPSd");
        return false;
      }

      RCLCPP_INFO(this->get_logger(), "GPSd opened");
      return true;
    }

    void step()
    {
#if GPSD_API_MAJOR_VERSION >= 5
      if (!gps_->waiting(1e6))
        return;

      // Read out all queued data and only act on the latest
      gps_data_t* p = NULL;
      while (gps_->waiting(0))
      {
        p = gps_->read();
      }

#else
      gps_data_t *p = gps->poll();
#endif
      process_data(p);
    }

    void stop()
    {
      // gpsmm doesn't have a close method? OK ...
    }

  private:
    void process_data(struct gps_data_t* p)
    {
      if (p == nullptr)
        return;

#if GPSD_API_MAJOR_VERSION >= 9
      if (!p->online.tv_sec && !p->online.tv_nsec) {
#else
      if (!p->online) {
#endif
        return;
      }

      process_data_gps(p);
      process_data_navsat(p);
    }


#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#else
#error "gpsd_client only supports gpsd API versions 3+"
#endif

    void process_data_gps(struct gps_data_t* p)
    {
      rclcpp::Time time = this->get_clock()->now();

      gps_msgs::msg::GPSFix fix;
      gps_msgs::msg::GPSStatus status;

      status.header.stamp = time;
      fix.header.stamp = time;
      fix.header.frame_id = frame_id_;

      status.satellites_used = p->satellites_used;

#if GPSD_API_MAJOR_VERSION > 5
      status.satellite_used_prn.clear();
      status.satellite_used_prn.reserve(p->satellites_used);
      for (int i = 0; i < p->satellites_visible; ++i)
      {
        if (p->skyview[i].used)
        {
          status.satellite_used_prn.push_back(p->skyview[i].PRN);
        }
      }
#else
      status.satellite_used_prn.resize(p->satellites_used);
      for (int i = 0; i < p->satellites_used; ++i)
      {
        status.satellite_used_prn[i] = p->used[i];
      }
#endif

      status.satellites_visible = SATS_VISIBLE;

      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);

      for (int i = 0; i < SATS_VISIBLE; ++i)
      {
#if GPSD_API_MAJOR_VERSION > 5
        status.satellite_visible_prn[i] = p->skyview[i].PRN;
        status.satellite_visible_z[i] = p->skyview[i].elevation;
        status.satellite_visible_azimuth[i] = p->skyview[i].azimuth;
        status.satellite_visible_snr[i] = p->skyview[i].ss;
#else
        status.satellite_visible_prn[i] = p->PRN[i];
        status.satellite_visible_z[i] = p->elevation[i];
        status.satellite_visible_azimuth[i] = p->azimuth[i];
        status.satellite_visible_snr[i] = p->ss[i];
#endif
      }
      const bool valid_variance =
        std::isfinite(p->fix.epx) &&
        std::isfinite(p->fix.epy) &&
        std::isfinite(p->fix.epv);
      if (((p->fix.mode == MODE_2D) || (p->fix.mode == MODE_3D)) && !(check_fix_by_variance_ && !valid_variance))
      {
        status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
        status.orientation_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
        status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;

        status.status = 0; //gps_msgs::msg::GPSStatus::STATUS_FIX;

        bool sbas_used = false;

#if GPSD_API_MAJOR_VERSION >= 9
        for (int i = 0; i < p->satellites_visible; ++i)
        {
          if (p->skyview[i].used)
          {
            if (p->skyview[i].gnssid == GNSSID_SBAS)
              sbas_used = true;
          }
        }
#endif

#if GPSD_API_MAJOR_VERSION >= 10
      switch (p->fix.status)
#else
      switch (p->status)
#endif
      {
#if defined(STATUS_DGPS_FIX) || defined(STATUS_DGPS)
#ifdef STATUS_DGPS_FIX
        case STATUS_DGPS_FIX:
#else
        case STATUS_DGPS:
#endif
          if (sbas_used)
            status.status = 1; //gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
          else
            status.status = 18; //gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX;
          break;
#endif
#ifdef STATUS_RTK_FIX
        case STATUS_RTK_FIX:
          status.status = 19; //gps_msgs::msg::GPSStatus::STATUS_RTK_FIX;
          break;
#endif
#ifdef STATUS_RTK_FLT
        case STATUS_RTK_FLT:
          status.status = 20; //gps_msgs::msg::GPSStatus::STATUS_RTK_FLOAT;
          break;
#endif
        default:
          break;
      }

#if GPSD_API_MAJOR_VERSION >= 9
        fix.time = (double)(p->fix.time.tv_sec) + ((double)(p->fix.time.tv_nsec)) / 1e9;
#else
        fix.time = p->fix.time;
#endif
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        if (p->fix.mode == MODE_3D)
        {
          fix.altitude = p->fix.altitude;
        }
        else
        {
          fix.altitude = std::nan("");
        }
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

#if GPSD_API_MAJOR_VERSION < 8
        fix.err = p->epe;
#else
        fix.err = p->fix.eph;
#endif
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;

        /* TODO: attitude */
      } else {
        status.status = -1; //gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
      }

      fix.status = status;

      RCLCPP_DEBUG(this->get_logger(), "Publishing gps fix...");
      gps_fix_pub_->publish(fix);
    }

    void process_data_navsat(struct gps_data_t* p)
    {
      sensor_msgs::msg::NavSatFix::UniquePtr fix = std::make_unique<sensor_msgs::msg::NavSatFix>();

      /* TODO: Support SBAS and other GBAS. */

#if GPSD_API_MAJOR_VERSION >= 9
      if (use_gps_time_ && (p->online.tv_sec || p->online.tv_nsec)) {
        fix->header.stamp = rclcpp::Time(p->fix.time.tv_sec, p->fix.time.tv_nsec);
#else
      if (use_gps_time_ && std::isfinite(p->fix.time)) {
        fix->header.stamp = rclcpp::Time(p->fix.time);
#endif
      }
      else {
        fix->header.stamp = this->get_clock()->now();
      }

      fix->header.frame_id = frame_id_;

      bool sbas_used = false;

#if GPSD_API_MAJOR_VERSION > 5
      fix->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN;
      for (int i = 0; i < p->satellites_visible; ++i)
      {
        if (p->skyview[i].used)
        {
          if (p->skyview[i].gnssid == GNSSID_GPS)
            fix->status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
          else if (p->skyview[i].gnssid == GNSSID_GLO)
            fix->status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
          else if (p->skyview[i].gnssid == GNSSID_BD)
            fix->status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
          else if (p->skyview[i].gnssid == GNSSID_GAL)
            fix->status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
          if (p->skyview[i].gnssid == GNSSID_SBAS)
            sbas_used = true;
        }
      }
#else
      fix->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
#endif

      if (p->fix.mode == MODE_2D || p->fix.mode == MODE_3D)
      {
        fix->status.status = 0; //sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        /* gpsmm pollutes the global namespace with STATUS_,
         * so we need to use the ROS message's integer values
         * for status.status
         */
#if GPSD_API_MAJOR_VERSION >= 10
        switch (p->fix.status)
#else
        switch (p->status)
#endif
        {
#if defined(STATUS_DGPS_FIX) || defined(STATUS_DGPS)
#ifdef STATUS_DGPS_FIX
          case STATUS_DGPS_FIX:
#else
          case STATUS_DGPS:
#endif
            if (sbas_used)
              fix->status.status = 1; //sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            else
              fix->status.status = 2; //sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
#endif
#ifdef STATUS_RTK_FIX
          case STATUS_RTK_FIX:
            fix->status.status = 2; //sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
#endif
#ifdef STATUS_RTK_FLT
          case STATUS_RTK_FLT:
            fix->status.status = 2; //sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            break;
#endif
          default:
            break;
        }
      }
      else
      {
        fix->status.status = -1; //sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      }

      fix->latitude = p->fix.latitude;
      fix->longitude = p->fix.longitude;
      fix->altitude = p->fix.altitude;

      /* gpsd reports status=OK even when there is no current fix,
       * as long as there has been a fix previously. Throw out these
       * fake results, which have NaN variance
       */
      if (check_fix_by_variance_ &&
          (!std::isfinite(p->fix.epx) ||
           !std::isfinite(p->fix.epy) ||
           !std::isfinite(p->fix.epv)))
      {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(),
          *this->get_clock(),
          1000,
          "GPS status was reported as OK, but variance was invalid");
        return;
      }

      fix->position_covariance[0] = p->fix.epx;
      fix->position_covariance[4] = p->fix.epy;
      fix->position_covariance[8] = p->fix.epv;

      fix->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

      RCLCPP_DEBUG(this->get_logger(), "Publishing navsatfix...");
      navsatfix_pub_->publish(std::move(fix));
    }

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_fix_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_pub_;

    gpsmm* gps_;

    bool use_gps_time_;
    bool check_fix_by_variance_;
    std::string frame_id_;
    int publish_rate_;
    std::chrono::milliseconds publish_period_ms{};
    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gpsd_client::GPSDClientComponent)
