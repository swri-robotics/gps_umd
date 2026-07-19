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

#include <gps.h>

#if GPSD_API_MAJOR_VERSION < 9
#error "gpsd_client requires gpsd API version >= 9 (gpsd >= 3.20)"
#endif

namespace gpsd_client
{

/// Options controlling how gpsd reports are converted into ROS messages.
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

/// Converts gpsd's gps_data_t reports into ROS messages.
///
/// Implementations are specific to a range of gpsd API versions and are
/// obtained through GpsdParserFactory. Each parser is named after the highest
/// GPSD_API_MAJOR_VERSION it supports.
class GpsdParser
{
public:
  virtual ~GpsdParser() = default;

  /// True if gpsd reports a device online for this report.
  [[nodiscard]] virtual bool isOnline(const gps_data_t& data) const = 0;

  /// Convert a report into a GPSFix message stamped with @p stamp.
  [[nodiscard]] virtual gps_msgs::msg::GPSFix parseGpsFix(const gps_data_t& data,
                                            const rclcpp::Time& stamp) const = 0;

  /// Convert a report into a NavSatFix message.
  ///
  /// The message is stamped with GPS time when the context enables
  /// use_gps_time, otherwise with @p fallback_stamp. Returns std::nullopt when
  /// the fix is rejected by the variance check and should not be published.
  [[nodiscard]] virtual std::optional<sensor_msgs::msg::NavSatFix> parseNavSatFix(
      const gps_data_t& data, const rclcpp::Time& fallback_stamp) const = 0;
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_PARSER_HPP_
