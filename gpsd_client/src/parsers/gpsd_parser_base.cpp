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

#include <gpsd_client/parsers/gpsd_parser_base.hpp>

#include <cmath>

#include <gps_msgs/msg/gps_status.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace gpsd_client
{
constexpr uint32_t NANOSECONDS_IN_SECOND = 1e9;

bool GpsdParserBase::isOnline(const gps_data_t & data) const
{
  return (data.online.tv_sec > 0) || (data.online.tv_nsec > 0);
}

bool GpsdParserBase::usedSbas(const gps_data_t & data)
{
  for (int i = 0; i < data.satellites_visible; ++i) {
    if (data.skyview[i].used && data.skyview[i].gnssid == GNSSID_SBAS) {
      return true;
    }
  }
  return false;
}

bool GpsdParserBase::sbasAugmented(const gps_data_t & data) const
{
  return context_.override_augmentation_source || usedSbas(data);
}

double GpsdParserBase::ellipsoidAltitude(const gps_data_t & data)
{
  /* gps_fix_t::altitude has been "DEPRECATED, undefined" since GPSd 3.20.
   * GPSd still sends the old "alt" key, but fills it from altMSL whenever it
   * has one, so it is height above the geoid. NavSatFix defines altitude as
   * height above the WGS 84 ellipsoid, which is altHAE; the two differ by the
   * geoid separation, which can be tens of meters.
   *
   * Only a 3D fix has an altitude. Both messages use NaN for "no altitude".
   */
  if (data.fix.mode != MODE_3D) {
    return std::nan("");
  }
  return data.fix.altHAE;
}

double GpsdParserBase::variance(double uncertainty) const
{
  /* GPSd's epx, epy and epv are error estimates in meters, not variances, and
   * GPSd does not say how confident they are ("Certainty unknown", in its JSON
   * documentation). Where they come from varies:
   *
   *   - GPSd's own estimate, used when the receiver gives none, is DOP times a
   *     user range error constant labelled "95% confidence" (libgpsd_core.c).
   *   - Some drivers scale the receiver's figure toward 95%: NavCom by 1.96,
   *     Garmin and Zodiac by CEP95_SIGMA (2.45).
   *   - Others pass the receiver's own figure through: NMEA GBS "expected
   *     error", AllyStar, SiRF.
   *
   * So the uncertainty is divided by uncertainty_to_sigma to give a standard
   * deviation, and squared. The default, 1.96, follows GPSd's 95% intent. A
   * receiver known to report standard deviations wants 1.0, which is also the
   * conservative choice: it overstates the variance of a 95% figure, where
   * 1.96 would understate the variance of a 1-sigma one.
   */
  const double sigma = uncertainty / context_.uncertainty_to_sigma;
  return sigma * sigma;
}

bool GpsdParserBase::hasValidVariance(const gps_data_t & data)
{
  return std::isfinite(data.fix.epx) &&
         std::isfinite(data.fix.epy) &&
         std::isfinite(data.fix.epv);
}

int16_t GpsdParserBase::mapGpsFixStatus(int gpsd_status, bool sbas_used)
{
  using gps_msgs::msg::GPSStatus;
  switch (gpsd_status) {
    case gps_h::kStatusDgps:
      return sbas_used ? GPSStatus::STATUS_SBAS_FIX : GPSStatus::STATUS_DGPS_FIX;
    case gps_h::kStatusRtkFix:
      return GPSStatus::STATUS_RTK_FIX;
    case gps_h::kStatusRtkFloat:
      return GPSStatus::STATUS_RTK_FLOAT;
    default:
      return GPSStatus::STATUS_FIX;
  }
}

int8_t GpsdParserBase::mapNavSatStatus(int gpsd_status, bool sbas_used)
{
  using sensor_msgs::msg::NavSatStatus;
  switch (gpsd_status) {
    case gps_h::kStatusDgps:
      return sbas_used ? NavSatStatus::STATUS_SBAS_FIX : NavSatStatus::STATUS_GBAS_FIX;
    case gps_h::kStatusRtkFix:
    case gps_h::kStatusRtkFloat:
      return NavSatStatus::STATUS_GBAS_FIX;
    default:
      return NavSatStatus::STATUS_FIX;
  }
}

gps_msgs::msg::GPSFix GpsdParserBase::parseGpsFix(
  const gps_data_t & data,
  const rclcpp::Time & stamp) const
{
  gps_msgs::msg::GPSFix fix;
  gps_msgs::msg::GPSStatus status;

  status.header.stamp = stamp;
  fix.header.stamp = stamp;
  fix.header.frame_id = context_.frame_id;

  status.satellites_used = data.satellites_used;

  status.satellite_used_prn.reserve(data.satellites_used);
  for (int i = 0; i < data.satellites_visible; ++i) {
    if (data.skyview[i].used) {
      status.satellite_used_prn.push_back(data.skyview[i].PRN);
    }
  }

  status.satellites_visible = data.satellites_visible;

  status.satellite_visible_prn.resize(status.satellites_visible);
  status.satellite_visible_z.resize(status.satellites_visible);
  status.satellite_visible_azimuth.resize(status.satellites_visible);
  status.satellite_visible_snr.resize(status.satellites_visible);

  for (int i = 0; i < data.satellites_visible; ++i) {
    status.satellite_visible_prn[i] = data.skyview[i].PRN;
    status.satellite_visible_z[i] = data.skyview[i].elevation;
    status.satellite_visible_azimuth[i] = data.skyview[i].azimuth;
    status.satellite_visible_snr[i] = data.skyview[i].ss;
  }

  if (((data.fix.mode == MODE_2D) || (data.fix.mode == MODE_3D)) &&
    (!context_.check_fix_by_variance || hasValidVariance(data)))
  {
    status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
    status.orientation_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
    status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;

    status.status = mapGpsFixStatus(getFixStatus(data), sbasAugmented(data));

    fix.time = static_cast<double>(data.fix.time.tv_sec) +
      (static_cast<double>(data.fix.time.tv_nsec) / NANOSECONDS_IN_SECOND);
    fix.latitude = data.fix.latitude;
    fix.longitude = data.fix.longitude;
    fix.altitude = ellipsoidAltitude(data);
    fix.track = data.fix.track;
    fix.speed = data.fix.speed;
    fix.climb = data.fix.climb;

    fix.pdop = data.dop.pdop;
    fix.hdop = data.dop.hdop;
    fix.vdop = data.dop.vdop;
    fix.tdop = data.dop.tdop;
    fix.gdop = data.dop.gdop;

    if (hasValidVariance(data)) {
      fix.position_covariance[0] = variance(data.fix.epx);
      fix.position_covariance[4] = variance(data.fix.epy);
      fix.position_covariance[8] = variance(data.fix.epv);
      fix.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    } else {
      fix.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN;
    }

    fix.err = data.fix.eph;
    fix.err_vert = data.fix.epv;
    fix.err_track = data.fix.epd;
    fix.err_speed = data.fix.eps;
    fix.err_climb = data.fix.epc;
    fix.err_time = data.fix.ept;

    /* TODO: attitude */
  } else {
    status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
  }

  fix.status = status;

  return fix;
}

std::optional<sensor_msgs::msg::NavSatFix> GpsdParserBase::parseNavSatFix(
  const gps_data_t & data, const rclcpp::Time & fallback_stamp) const
{
  sensor_msgs::msg::NavSatFix fix;

  /* TODO: Support SBAS and other GBAS. */

  /* GPSd leaves the fix time unset when the receiver has no time to give,
   * which a receiver without a fix often does not. Stamping with it would put
   * the message at the epoch.
   */
  const bool have_gps_time = (data.fix.time.tv_sec > 0) || (data.fix.time.tv_nsec > 0);
  if (context_.use_gps_time && have_gps_time &&
    ((data.online.tv_sec > 0) || (data.online.tv_nsec > 0)))
  {
    fix.header.stamp = rclcpp::Time(
      static_cast<uint32_t>(data.fix.time.tv_sec),
      static_cast<uint32_t>(data.fix.time.tv_nsec));
  } else {
    fix.header.stamp = fallback_stamp;
  }

  fix.header.frame_id = context_.frame_id;

#ifdef NO_UNKNOWN_FIX
  fix.status.service = 0;  // Initialize to 0 before setting bits
#else
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN;
#endif
  for (int i = 0; i < data.satellites_visible; ++i) {
    if (data.skyview[i].used) {
      if (data.skyview[i].gnssid == GNSSID_GPS) {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      } else if (data.skyview[i].gnssid == GNSSID_GLO) {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
      } else if (data.skyview[i].gnssid == GNSSID_BD) {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
      } else if (data.skyview[i].gnssid == GNSSID_GAL) {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
      }
    }
  }

  if (data.fix.mode == MODE_2D || data.fix.mode == MODE_3D) {
    fix.status.status = mapNavSatStatus(getFixStatus(data), sbasAugmented(data));
  } else {
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  }

  fix.latitude = data.fix.latitude;
  fix.longitude = data.fix.longitude;
  fix.altitude = ellipsoidAltitude(data);

  /* GPSd reports status=OK even when there is no current fix, as long as
   * there has been a fix previously. Throw out these fake results, which
   * have NaN variance.
   */
  if (context_.check_fix_by_variance && !hasValidVariance(data)) {
    return std::nullopt;
  }

  /* Covariance is a 3x3 matrix, and this sets the diagonal elements from the
   * reported uncertainties; see variance(). GPSd reports an uncertainty it
   * does not have as NaN, so the matrix is only advertised as known when all
   * three are finite; otherwise it stays zero-filled and UNKNOWN, since
   * NavSatFix has no way to mark individual elements as missing and
   * downstream consumers are entitled to treat a KNOWN covariance as usable
   * numbers. This matters when check_fix_by_variance is off because when it is
   * on, a fix with a NaN uncertainty has already been dropped above.
   */
  if (hasValidVariance(data)) {
    // legacy_fix_semantics keeps what earlier releases published: the
    // uncertainties themselves, in meters, rather than variances.
    const bool legacy = context_.legacy_fix_semantics;
    fix.position_covariance[0] = legacy ? data.fix.epx : variance(data.fix.epx);
    fix.position_covariance[4] = legacy ? data.fix.epy : variance(data.fix.epy);
    fix.position_covariance[8] = legacy ? data.fix.epv : variance(data.fix.epv);

    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  return fix;
}

}  // namespace gpsd_client
