/*
 * Translates nav_msgs/Odometry in UTM coordinates back into sensor_msgs/NavSat{Fix,Status}
 * Useful for visualizing UTM data on a map or comparing with raw GPS data
 * Added by Dheera Venkatraman (dheera@dheera.net)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <gps_tools/conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>

namespace gps_tools
{
class UtmOdometryToNavSatFixComponent : public rclcpp::Node
{
public:
  explicit UtmOdometryToNavSatFixComponent(const rclcpp::NodeOptions& options) :
      Node("utm_odometry_to_navsatfix_node", options)
  {

    frame_id_ = declare_parameter("frame_id", std::string(""));
    try
    {
        zone_ = declare_parameter<int>("zone");
    }
    catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException)
    {
        // If zone is not set, just leave it with its default value (std::nullopt).
    }

    fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);

    auto callback = [this](const typename nav_msgs::msg::Odometry::SharedPtr odom) -> void
    {
      if (odom->header.stamp.sec == 0 && odom->header.stamp.nanosec == 0) {
        return;
      }

      if (!fix_pub_) {
        return;
      }

      double northing, easting, latitude, longitude;
      std::string zone;
      sensor_msgs::msg::NavSatFix fix;

      northing = odom->pose.pose.position.y;
      easting = odom->pose.pose.position.x;

      if(zone_) {
        // utm zone was supplied as a ROS parameter
        zone = std::to_string(zone_.value());
        fix.header.frame_id = odom->header.frame_id;
      } else {
        // look for the utm zone in the frame_id
        std::size_t pos = odom->header.frame_id.find("/utm_");
        if(pos==std::string::npos) {
          RCLCPP_WARN(this->get_logger(), "UTM zone not found in frame_id");
          return;
        }
        zone = odom->header.frame_id.substr(pos + 5, 3);
        fix.header.frame_id = odom->header.frame_id.substr(0, pos);
      }

      RCLCPP_INFO(this->get_logger(), "zone: %s", zone.c_str());

      fix.header.stamp = odom->header.stamp;

      UTMtoLL(northing, easting, zone, latitude, longitude);

      fix.latitude = latitude;
      fix.longitude = longitude;
      fix.altitude = odom->pose.pose.position.z;

      fix.position_covariance[0] = odom->pose.covariance[0];
      fix.position_covariance[1] = odom->pose.covariance[1];
      fix.position_covariance[2] = odom->pose.covariance[2];
      fix.position_covariance[3] = odom->pose.covariance[6];
      fix.position_covariance[4] = odom->pose.covariance[7];
      fix.position_covariance[5] = odom->pose.covariance[8];
      fix.position_covariance[6] = odom->pose.covariance[12];
      fix.position_covariance[7] = odom->pose.covariance[13];
      fix.position_covariance[8] = odom->pose.covariance[14];

      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

      fix_pub_->publish(fix);
    };
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, callback);
  }
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

  std::string frame_id_;
  std::optional<int> zone_;
};
}

/*using namespace gps_tools;

static ros::Publisher fix_pub;
std::string frame_id, child_frame_id;
std::string zone_param;
double rot_cov;

void callback(const nav_msgs::OdometryConstPtr& odom) {

  if (odom->header.stamp == ros::Time(0)) {
    return;
  }

  if (!fix_pub) {
    return;
  }

  double northing, easting, latitude, longitude;
  std::string zone;
  sensor_msgs::NavSatFix fix;

  northing = odom->pose.pose.position.y;
  easting = odom->pose.pose.position.x;

  if(zone_param.length() > 0) {
    // utm zone was supplied as a ROS parameter
    zone = zone_param;
    fix.header.frame_id = odom->header.frame_id;
  } else {
    // look for the utm zone in the frame_id
    std::size_t pos = odom->header.frame_id.find("/utm_");
    if(pos==std::string::npos) {
      ROS_WARN("UTM zone not found in frame_id");
      return;
    }
    zone = odom->header.frame_id.substr(pos + 5, 3);
    fix.header.frame_id = odom->header.frame_id.substr(0, pos);
  }

  ROS_INFO("zone: %s", zone.c_str());

  fix.header.stamp = odom->header.stamp;

  UTMtoLL(northing, easting, zone, latitude, longitude);

  fix.latitude = latitude;
  fix.longitude = longitude; 
  fix.altitude = odom->pose.pose.position.z;

  fix.position_covariance[0] = odom->pose.covariance[0];
  fix.position_covariance[1] = odom->pose.covariance[1];
  fix.position_covariance[2] = odom->pose.covariance[2];
  fix.position_covariance[3] = odom->pose.covariance[6];
  fix.position_covariance[4] = odom->pose.covariance[7];
  fix.position_covariance[5] = odom->pose.covariance[8];
  fix.position_covariance[6] = odom->pose.covariance[12];
  fix.position_covariance[7] = odom->pose.covariance[13];
  fix.position_covariance[8] = odom->pose.covariance[14];
  
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  fix_pub.publish(fix);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_to_navsatfix_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("zone", zone_param, "");

  fix_pub = node.advertise<sensor_msgs::NavSatFix>("odom_fix", 10);

  ros::Subscriber odom_sub = node.subscribe("odom", 10, callback);

  ros::spin();
}
*/

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UtmOdometryToNavSatFixComponent)
