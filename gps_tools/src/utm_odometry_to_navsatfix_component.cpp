// *****************************************************************************
//
// Copyright (c) 2017, Dheera Venkatraman
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the copyright holder nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

/*
 * Translates nav_msgs/Odometry in UTM coordinates back into sensor_msgs/NavSat{Fix,Status}
 * Useful for visualizing UTM data on a map or comparing with raw GPS data
 * Added by Dheera Venkatraman (dheera@dheera.net)
 */

#include <cctype>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include "gps_tools/conversions.h"
#include <nav_msgs/msg/odometry.hpp>

namespace gps_tools
{
class UtmOdometryToNavSatFixComponent : public rclcpp::Node
{
public:
  explicit UtmOdometryToNavSatFixComponent(const rclcpp::NodeOptions & options)
  : Node("utm_odometry_to_navsatfix_node", options)
  {
    frame_id_ = declare_parameter("frame_id", std::string(""));

    /* A full zone designator such as "14R" says which hemisphere the
     * northings are in; a zone number alone does not. An integer is still
     * accepted, as it has been since the parameter was first declared, and
     * means that zone in the northern hemisphere.
     */
    rcl_interfaces::msg::ParameterDescriptor zone_descriptor;
    zone_descriptor.description =
      "UTM zone of the odometry, such as \"14R\". A zone number alone means the "
      "northern hemisphere. Unset, the zone comes from the frame_id.";
    zone_descriptor.dynamic_typing = true;
    const rclcpp::ParameterValue zone =
      declare_parameter("zone", rclcpp::ParameterValue(), zone_descriptor);
    if (zone.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      zone_ = zone.get<std::string>();
    } else if (zone.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      zone_ = std::to_string(zone.get<int64_t>());
    } else if (zone.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
      RCLCPP_ERROR(
        get_logger(), "zone must be a string such as \"14R\" or an integer; ignoring it");
    }
    // UTMtoLL() reads a missing band letter as the southern hemisphere.
    if (zone_ && !zone_->empty() && std::isdigit(static_cast<unsigned char>(zone_->back()))) {
      zone_->push_back('N');
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

        if (zone_) {
          // utm zone was supplied as a ROS parameter
          zone = zone_.value();
          fix.header.frame_id = odom->header.frame_id;
        } else {
          // look for the utm zone in the frame_id
          std::size_t pos = odom->header.frame_id.find("/utm_");
          if (pos == std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "UTM zone not found in frame_id");
            return;
          }
          zone = odom->header.frame_id.substr(pos + 5, 3);
          fix.header.frame_id = odom->header.frame_id.substr(0, pos);
        }

        if (!frame_id_.empty()) {
          fix.header.frame_id = frame_id_;
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
  std::optional<std::string> zone_;
};
}  // namespace gps_tools

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
