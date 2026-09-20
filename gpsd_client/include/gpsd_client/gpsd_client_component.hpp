#ifndef GPSD_CLIENT_GPSD_CLIENT_COMPONENT_HPP
#define GPSD_CLIENT_GPSD_CLIENT_COMPONENT_HPP

#include <rclcpp/rclcpp.hpp>

#include <gpsd_client/gpsd_client_base.hpp>

namespace gpsd_client
{
/* The unmanaged node: connects and starts publishing as soon as it is
 * constructed, and stays that way until it is destroyed.
 *
 * For the managed equivalent, whose connection and publishing are driven by
 * lifecycle transitions, see GPSDClientLifecycleComponent.
 */
class GPSDClientComponent : public GPSDClientBase<rclcpp::Node>
{
public:
  explicit GPSDClientComponent(const rclcpp::NodeOptions & options)
  : GPSDClientBase<rclcpp::Node>(options)
  {
    if (!doConfigure() || !doActivate())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to start gpsd_client; timer not created.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Instantiated.");
  }
};
}  // namespace gpsd_client

#endif  // GPSD_CLIENT_GPSD_CLIENT_COMPONENT_HPP
