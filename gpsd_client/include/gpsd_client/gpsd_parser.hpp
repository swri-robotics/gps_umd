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

#ifndef GPSD_CLIENT__GPSD_PARSER_HPP_
#define GPSD_CLIENT__GPSD_PARSER_HPP_

#include <optional>
#include <string>

// NOTE: gps.h pollutes the global namespace with STATUS_* macros that
// collide with the ROS message constants of the same names, so the message
// headers must be included before it.
#include <gps_msgs/msg/gps_fix.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <gps.h>  // NOLINT(build/include_order)

#if GPSD_API_MAJOR_VERSION < 9
#error "gpsd_client requires GPSd API version >= 9 (GPSd >= 3.20)"
#endif

namespace gpsd_client
{

/// Options controlling how GPSd reports are converted into ROS messages.
struct ParserContext
{
  std::string frame_id;
  bool use_gps_time;
  bool check_fix_by_variance;
  /// Report a DGPS fix as SBAS regardless of whether an SBAS satellite was
  /// used in the solution. Some receivers apply SBAS corrections without
  /// listing the SBAS satellite in the skyview.
  bool override_augmentation_source;
};

/// Converts GPSd's gps_data_t reports into ROS messages.
///
/// Implementations are specific to a range of GPSd API versions and are
/// obtained through GpsdParserFactory. Each parser is named after the highest
/// GPSD_API_MAJOR_VERSION it supports.
class GpsdParser
{
public:
  virtual ~GpsdParser() = default;

  /// True if GPSd reports a device online for this report.
  [[nodiscard]] virtual bool isOnline(const gps_data_t & data) const = 0;

  /// Convert a report into a GPSFix message stamped with @p stamp.
  [[nodiscard]] virtual gps_msgs::msg::GPSFix parseGpsFix(
    const gps_data_t & data,
    const rclcpp::Time & stamp) const = 0;

  /// Convert a report into a NavSatFix message.
  ///
  /// The message is stamped with GPS time when the context enables
  /// use_gps_time, otherwise with @p fallback_stamp. Returns std::nullopt when
  /// the fix is rejected by the variance check and should not be published.
  [[nodiscard]] virtual std::optional<sensor_msgs::msg::NavSatFix> parseNavSatFix(
    const gps_data_t & data, const rclcpp::Time & fallback_stamp) const = 0;
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_PARSER_HPP_
