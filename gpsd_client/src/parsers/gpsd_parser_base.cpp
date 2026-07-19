#include <gpsd_client/parsers/gpsd_parser_base.hpp>

#include <cmath>

#include <gps_msgs/msg/gps_status.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

namespace gpsd_client
{
constexpr uint32_t NANOSECONDS_IN_SECOND = 1e9;

bool GpsdParserBase::isOnline(const gps_data_t& data) const
{
  return ((data.online.tv_sec > 0) || (data.online.tv_nsec > 0));
}

bool GpsdParserBase::usedSbas(const gps_data_t& data)
{
  for (int i = 0; i < data.satellites_visible; ++i)
  {
    if (data.skyview[i].used && data.skyview[i].gnssid == GNSSID_SBAS)
    {
      return true;
    }
  }
  return false;
}

bool GpsdParserBase::sbasAugmented(const gps_data_t& data) const
{
  return context_.override_augmentation_source || usedSbas(data);
}

bool GpsdParserBase::hasValidVariance(const gps_data_t& data)
{
  return std::isfinite(data.fix.epx) &&
         std::isfinite(data.fix.epy) &&
         std::isfinite(data.fix.epv);
}

/* gpsmm pollutes the global namespace with STATUS_, so we need to use the
 * ROS messages' integer values for status.status in the mapping helpers
 * below.
 *
 * gpsd renamed STATUS_DGPS_FIX to STATUS_DGPS in 3.25. The rename was clean
 * -- no release defines both -- so the spelling has to be selected by the
 * preprocessor here, since it is used as a case label. The other STATUS_
 * macros are defined by every gpsd we support (API >= 9, enforced in
 * gpsd_parser.hpp) and need no feature detection.
 */
#ifdef STATUS_DGPS_FIX
#define GPSD_STATUS_DGPS STATUS_DGPS_FIX
#else
#define GPSD_STATUS_DGPS STATUS_DGPS
#endif

int16_t GpsdParserBase::mapGpsFixStatus(int gpsd_status, bool sbas_used)
{
  switch (gpsd_status)
  {
    case GPSD_STATUS_DGPS:
      if (sbas_used)
      {
        return 1;  // gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX
      }
      else
      {
        return 18; // gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX
      }
    case STATUS_RTK_FIX:
      return 19;   // gps_msgs::msg::GPSStatus::STATUS_RTK_FIX
    case STATUS_RTK_FLT:
      return 20;   // gps_msgs::msg::GPSStatus::STATUS_RTK_FLOAT
    default:
      return 0;    // gps_msgs::msg::GPSStatus::STATUS_FIX
  }
}

int8_t GpsdParserBase::mapNavSatStatus(int gpsd_status, bool sbas_used)
{
  switch (gpsd_status)
  {
    case GPSD_STATUS_DGPS:
      if (sbas_used)
      {
        return 1;  // sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX
      }
      else
      {
        return 2;  // sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
      }
    case STATUS_RTK_FIX:
    case STATUS_RTK_FLT:
      return 2;    // sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX
    default:
      return 0;    // sensor_msgs::msg::NavSatStatus::STATUS_FIX
  }
}

gps_msgs::msg::GPSFix GpsdParserBase::parseGpsFix(const gps_data_t& data,
                                                  const rclcpp::Time& stamp) const
{
  gps_msgs::msg::GPSFix fix;
  gps_msgs::msg::GPSStatus status;

  status.header.stamp = stamp;
  fix.header.stamp = stamp;
  fix.header.frame_id = context_.frame_id;

  status.satellites_used = data.satellites_used;

  status.satellite_used_prn.reserve(data.satellites_used);
  for (int i = 0; i < data.satellites_visible; ++i)
  {
    if (data.skyview[i].used)
    {
      status.satellite_used_prn.push_back(data.skyview[i].PRN);
    }
  }

  status.satellites_visible = data.satellites_visible;

  status.satellite_visible_prn.resize(status.satellites_visible);
  status.satellite_visible_z.resize(status.satellites_visible);
  status.satellite_visible_azimuth.resize(status.satellites_visible);
  status.satellite_visible_snr.resize(status.satellites_visible);

  for (int i = 0; i < data.satellites_visible; ++i)
  {
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
    if (data.fix.mode == MODE_3D)
    {
      fix.altitude = data.fix.altitude;
    }
    else
    {
      fix.altitude = std::nan("");
    }
    fix.track = data.fix.track;
    fix.speed = data.fix.speed;
    fix.climb = data.fix.climb;

    fix.pdop = data.dop.pdop;
    fix.hdop = data.dop.hdop;
    fix.vdop = data.dop.vdop;
    fix.tdop = data.dop.tdop;
    fix.gdop = data.dop.gdop;

    fix.err = data.fix.eph;
    fix.err_vert = data.fix.epv;
    fix.err_track = data.fix.epd;
    fix.err_speed = data.fix.eps;
    fix.err_climb = data.fix.epc;
    fix.err_time = data.fix.ept;

    /* TODO: attitude */
  }
  else
  {
    status.status = -1; // gps_msgs::msg::GPSStatus::STATUS_NO_FIX
  }

  fix.status = status;

  return fix;
}

std::optional<sensor_msgs::msg::NavSatFix> GpsdParserBase::parseNavSatFix(
    const gps_data_t& data, const rclcpp::Time& fallback_stamp) const
{
  sensor_msgs::msg::NavSatFix fix;

  /* TODO: Support SBAS and other GBAS. */

  if (context_.use_gps_time && ((data.online.tv_sec > 0) || (data.online.tv_nsec > 0)))
  {
    fix.header.stamp = rclcpp::Time(
      static_cast<uint32_t>(data.fix.time.tv_sec),
      static_cast<uint32_t>(data.fix.time.tv_nsec));
  }
  else
  {
    fix.header.stamp = fallback_stamp;
  }

  fix.header.frame_id = context_.frame_id;

#ifdef NO_UNKNOWN_FIX
  fix.status.service = 0; // Initialize to 0 before setting bits
#else
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN;
#endif
  for (int i = 0; i < data.satellites_visible; ++i)
  {
    if (data.skyview[i].used)
    {
      if (data.skyview[i].gnssid == GNSSID_GPS)
      {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      }
      else if (data.skyview[i].gnssid == GNSSID_GLO)
      {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
      }
      else if (data.skyview[i].gnssid == GNSSID_BD)
      {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
      }
      else if (data.skyview[i].gnssid == GNSSID_GAL)
      {
        fix.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
      }
    }
  }

  if (data.fix.mode == MODE_2D || data.fix.mode == MODE_3D)
  {
    fix.status.status = mapNavSatStatus(getFixStatus(data), sbasAugmented(data));
  }
  else
  {
    fix.status.status = -1; // sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
  }

  fix.latitude = data.fix.latitude;
  fix.longitude = data.fix.longitude;
  fix.altitude = data.fix.altitude;

  /* gpsd reports status=OK even when there is no current fix, as long as
   * there has been a fix previously. Throw out these fake results, which
   * have NaN variance.
   */
  if (context_.check_fix_by_variance && !hasValidVariance(data))
  {
    return std::nullopt;
  }

  // Covariance is a 3x3 matrix, and this sets the diagonal elements based on the reported variances.
  fix.position_covariance[0] = data.fix.epx;
  fix.position_covariance[4] = data.fix.epy;
  fix.position_covariance[8] = data.fix.epv;

  fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return fix;
}

}  // namespace gpsd_client
