#ifndef GPSD_CLIENT__PARSERS__GPSD_PARSER_BASE_HPP_
#define GPSD_CLIENT__PARSERS__GPSD_PARSER_BASE_HPP_

#include <cstdint>
#include <optional>

#include <gpsd_client/gpsd_parser.hpp>

namespace gpsd_client
{

/// Shared parsing implementation for gpsd APIs 9 and newer.
///
/// Everything here uses only gps_data_t fields whose layout is identical
/// across the supported API range (timespec online/fix.time, skyview[],
/// gnssid, the dop struct, fix.eph). The only field that moved between
/// versions is where the fix status lives, which concrete parsers provide
/// via getFixStatus().
///
/// Separately, gpsd renamed the STATUS_DGPS_FIX macro in 3.25; because it is
/// used as a case label, that spelling is resolved by the preprocessor in
/// gpsd_parser_base.cpp rather than through this class hierarchy.
class GpsdParserBase : public GpsdParser
{
public:
  explicit GpsdParserBase(ParserContext context)
    : context_(std::move(context))
  {
  }

  [[nodiscard]] bool isOnline(const gps_data_t& data) const override;

  [[nodiscard]] gps_msgs::msg::GPSFix parseGpsFix(const gps_data_t& data,
                                    const rclcpp::Time& stamp) const override;

  [[nodiscard]] std::optional<sensor_msgs::msg::NavSatFix> parseNavSatFix(
      const gps_data_t& data, const rclcpp::Time& fallback_stamp) const override;

protected:
  /// Fix status (STATUS_*): gps_data_t::status in API 9, moved to
  /// gps_data_t::fix.status in API 10.
  [[nodiscard]] virtual int getFixStatus(const gps_data_t& data) const = 0;

private:
  /// True if any satellite used in the solution is an SBAS satellite.
  static bool usedSbas(const gps_data_t& data);

  /// True if a DGPS report should be attributed to SBAS: either an SBAS
  /// satellite was used, or the context forces it.
  [[nodiscard]] bool sbasAugmented(const gps_data_t& data) const;

  /// True if epx/epy/epv are all finite.
  static bool hasValidVariance(const gps_data_t& data);

  static int16_t mapGpsFixStatus(int gpsd_status, bool sbas_used);

  static int8_t mapNavSatStatus(int gpsd_status, bool sbas_used);

  ParserContext context_;
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__PARSERS__GPSD_PARSER_BASE_HPP_
