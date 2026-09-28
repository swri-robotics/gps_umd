// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
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

#ifndef GPSD_CLIENT_GPSD_CLIENT_LIFECYCLE_COMPONENT_HPP
#define GPSD_CLIENT_GPSD_CLIENT_LIFECYCLE_COMPONENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <gpsd_client/gpsd_client_base.hpp>

namespace gpsd_client
{
/* The managed node: the same client as GPSDClientComponent, driven by
 * lifecycle transitions instead of by construction.
 *
 * The split follows libgps's own two phases. Configuring connects to GPSd --
 * so a missing or unreachable daemon fails the configure transition, rather
 * than leaving a node that exists but never publishes -- and activating asks
 * it to stream. Deactivating stops the stream and the timer but holds the
 * connection open, so a deactivate/activate pair costs no reconnect; only a
 * cleanup closes the socket.
 *
 * Publishers are created by the lifecycle node, which activates and
 * deactivates them along with the node itself.
 */
class GPSDClientLifecycleComponent : public GPSDClientBase<rclcpp_lifecycle::LifecycleNode>
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

  explicit GPSDClientLifecycleComponent(const rclcpp::NodeOptions & options)
  : GPSDClientBase<rclcpp_lifecycle::LifecycleNode>(options)
  {
    RCLCPP_INFO(this->get_logger(), "Instantiated; waiting to be configured.");
  }

  CallbackReturn on_configure(const State & /* previous_state */) override
  {
    if (!doConfigure()) {
      // Drop whatever was built before the failure; the node stays unconfigured.
      doCleanup();
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const State & previous_state) override
  {
    // Enables the managed publishers; without it they drop what is published.
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

    if (!doActivate()) {
      rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const State & previous_state) override
  {
    doDeactivate();
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const State & /* previous_state */) override
  {
    doCleanup();
    return CallbackReturn::SUCCESS;
  }

  /* Reachable from any state, including active, so it does both halves.
   * Both are safe to call on a node that never got that far.
   */
  CallbackReturn on_shutdown(const State & /* previous_state */) override
  {
    doDeactivate();
    doCleanup();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const State & /* previous_state */) override
  {
    doDeactivate();
    doCleanup();
    return CallbackReturn::SUCCESS;
  }
};
}  // namespace gpsd_client

#endif  // GPSD_CLIENT_GPSD_CLIENT_LIFECYCLE_COMPONENT_HPP
