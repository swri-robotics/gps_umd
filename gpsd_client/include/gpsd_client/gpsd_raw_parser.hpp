#ifndef GPSD_CLIENT__GPSD_RAW_PARSER_HPP_
#define GPSD_CLIENT__GPSD_RAW_PARSER_HPP_

#include <optional>

#include <gpsd_client/gpsd_parser.hpp>
#include <gpsd_client/gpsd_raw_message.hpp>

namespace gpsd_client
{

/// Converts a GPSd report into the raw message for this build's libgps API.
///
/// Unlike GpsdParser -- whose GPSFix/NavSatFix output is a curated,
/// version-independent view -- this is a near-verbatim copy of gps_data_t into
/// the GPSDRaw<MAJOR>v<MINOR> message matching the libgps the workspace was
/// built against. Which message that is comes from the single ladder in
/// gpsd_raw_message.hpp; nothing here or downstream repeats the version logic.
///
/// There is one implementation rather than one per API pair: the per-pair part
/// is the generated fill() in gpsd_client/parsers/generated/, selected by the
/// same ladder. What is left for hand-written code is the part a generator
/// cannot do safely -- the ROS header, and the variable-length arrays, whose
/// valid element counts live in sibling fields rather than in the type.
class GpsdRawParser
{
public:
  explicit GpsdRawParser(ParserContext context)
    : context_(std::move(context))
  {
  }

  /// Convert a report into the raw message, stamped with @p stamp.
  [[nodiscard]] GpsdRawMsg parseRaw(const gps_data_t& data,
                                    const rclcpp::Time& stamp) const;

  /// Convert an RTCM2 report, or nullopt when this report is not one.
  ///
  /// RTCM lives on its own topic rather than inside GPSDRaw: between them the
  /// two RTCM message families are 452 of the ~905 generated types, and they
  /// interest a quite different audience from a position fix.
  ///
  /// Returns nullopt unless the report's `set` mask names RTCM2. gps_data_t
  /// packs the report arms into a union, so the mask is the only thing that
  /// makes reading this arm defined rather than a reinterpretation of whatever
  /// arm was last written.
  [[nodiscard]] std::optional<GpsdRtcm2Msg> parseRtcm2(
      const gps_data_t& data, const rclcpp::Time& stamp) const;

  /// Convert an RTCM3 report, or nullopt when this report is not one.
  [[nodiscard]] std::optional<GpsdRtcm3Msg> parseRtcm3(
      const gps_data_t& data, const rclcpp::Time& stamp) const;

private:
  /// Number of skyview entries that are actually populated.
  ///
  /// gps_data_t::satellites_visible is a plain int that a partially-filled or
  /// stale report can leave negative or larger than the array, so it is
  /// clamped to [0, MAXCHANNELS] rather than trusted. Publishing the whole
  /// fixed array instead would emit MAXCHANNELS (140 or 184, depending on the
  /// GPSd) entries of uninitialised satellites.
  static std::size_t skyviewCount(const gps_data_t& data);

  /// Number of populated entries in gps_data_t::devices.list.
  ///
  /// Same reasoning as skyviewCount(): ndevices is a plain int and the array
  /// is MAXUSERDEVS long regardless of how much of it means anything.
  static std::size_t deviceCount(const gps_data_t& data);

  ParserContext context_;
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_RAW_PARSER_HPP_
