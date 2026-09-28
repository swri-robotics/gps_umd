// *****************************************************************************
//
// Copyright (c) 2010, Ken Tossell
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
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include "gps_tools/conversions.h"
#include <nav_msgs/msg/odometry.hpp>


namespace gps_tools
{
class UtmOdometryComponent : public rclcpp::Node
{
public:
  explicit UtmOdometryComponent(const rclcpp::NodeOptions & options)
  : Node("utm_odometry_node", options),
    fix_sub_(nullptr),
    rot_cov_(99999.0),
    append_zone_(false)
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    rot_cov_ = declare_parameter("rot_covariance", rot_cov_);
    child_frame_id_ = declare_parameter("child_frame_id", child_frame_id_);
    frame_id_ = declare_parameter("frame_id", frame_id_);
    append_zone_ = declare_parameter("append_zone", append_zone_);

    auto callback = [this](const typename sensor_msgs::msg::NavSatFix::SharedPtr fix) -> void
      {
        if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
          RCLCPP_DEBUG(this->get_logger(), "No fix.");
          return;
        }

        if (fix->header.stamp.nanosec == 0 && fix->header.stamp.sec == 0) {
          return;
        }

        double northing, easting;
        std::string zone;

        LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

        if (odom_pub_) {
          nav_msgs::msg::Odometry odom;
          odom.header.stamp = fix->header.stamp;

          if (frame_id_.empty()) {
            if (append_zone_) {
              odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
            } else {
              odom.header.frame_id = fix->header.frame_id;
            }
          } else {
            if (append_zone_) {
              odom.header.frame_id = frame_id_ + "/utm_" + zone;
            } else {
              odom.header.frame_id = frame_id_;
            }
          }

          odom.child_frame_id = child_frame_id_;

          odom.pose.pose.position.x = easting;
          odom.pose.pose.position.y = northing;
          odom.pose.pose.position.z = fix->altitude;

          odom.pose.pose.orientation.x = 0;
          odom.pose.pose.orientation.y = 0;
          odom.pose.pose.orientation.z = 0;
          odom.pose.pose.orientation.w = 1;

          // Use ENU covariance to build XYZRPY covariance
          std::array<double, 36> covariance = {{
            fix->position_covariance[0],
            fix->position_covariance[1],
            fix->position_covariance[2],
            0, 0, 0,
            fix->position_covariance[3],
            fix->position_covariance[4],
            fix->position_covariance[5],
            0, 0, 0,
            fix->position_covariance[6],
            fix->position_covariance[7],
            fix->position_covariance[8],
            0, 0, 0,
            0, 0, 0, rot_cov_, 0, 0,
            0, 0, 0, 0, rot_cov_, 0,
            0, 0, 0, 0, 0, rot_cov_
          }};

          odom.pose.covariance = covariance;

          odom_pub_->publish(odom);
        }
      };
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("fix", 10, callback);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

  std::string frame_id_;
  std::string child_frame_id_;
  double rot_cov_;
  bool append_zone_;
};
}  // namespace gps_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gps_tools::UtmOdometryComponent)
