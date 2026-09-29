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

#ifndef GPSD_CLIENT__PARSERS__GPSD_PARSER_BASE_HPP_
#define GPSD_CLIENT__PARSERS__GPSD_PARSER_BASE_HPP_

#include <cstdint>
#include <optional>
#include <utility>

#include <gpsd_client/gpsd_parser.hpp>

namespace gpsd_client
{

/// Shared parsing implementation for GPSd APIs 9 and newer.
///
/// Everything here uses only gps_data_t fields whose layout is identical
/// across the supported API range (timespec online/fix.time, skyview[],
/// gnssid, the dop struct, fix.eph). The only field that moved between
/// versions is where the fix status lives, which concrete parsers provide
/// via getFixStatus().
///
/// Separately, GPSd renamed some of the fix status macros; gpsd_client/gps.hpp
/// gives each value one name across the renames.
class GpsdParserBase : public GpsdParser
{
public:
  explicit GpsdParserBase(ParserContext context)
  : context_(std::move(context))
  {
  }

  [[nodiscard]] bool isOnline(const gps_data_t & data) const override;

  [[nodiscard]] gps_msgs::msg::GPSFix parseGpsFix(
    const gps_data_t & data,
    const rclcpp::Time & stamp) const override;

  [[nodiscard]] std::optional<sensor_msgs::msg::NavSatFix> parseNavSatFix(
    const gps_data_t & data, const rclcpp::Time & fallback_stamp) const override;

protected:
  /// Fix status (STATUS_*): gps_data_t::status in API 9, moved to
  /// gps_data_t::fix.status in API 10.
  [[nodiscard]] virtual int getFixStatus(const gps_data_t & data) const = 0;

private:
  /// True if any satellite used in the solution is an SBAS satellite.
  static bool usedSbas(const gps_data_t & data);

  /// True if a DGPS report should be attributed to SBAS: either an SBAS
  /// satellite was used, or the context forces it.
  [[nodiscard]] bool sbasAugmented(const gps_data_t & data) const;

  /// True if epx/epy/epv are all finite.
  static bool hasValidVariance(const gps_data_t & data);

  static int16_t mapGpsFixStatus(int gpsd_status, bool sbas_used);

  static int8_t mapNavSatStatus(int gpsd_status, bool sbas_used);

  ParserContext context_;
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__PARSERS__GPSD_PARSER_BASE_HPP_
