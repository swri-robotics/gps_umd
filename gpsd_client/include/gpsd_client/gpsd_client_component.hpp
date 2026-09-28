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

#ifndef GPSD_CLIENT__GPSD_CLIENT_COMPONENT_HPP_
#define GPSD_CLIENT__GPSD_CLIENT_COMPONENT_HPP_

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
    if (!doConfigure() || !doActivate()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start gpsd_client; timer not created.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Instantiated.");
  }
};
}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_CLIENT_COMPONENT_HPP_
