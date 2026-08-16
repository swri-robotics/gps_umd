#ifndef GPSD_CLIENT__GPSD_RAW_PARSER_HPP_
#define GPSD_CLIENT__GPSD_RAW_PARSER_HPP_

#include <gpsd_client/gpsd_parser.hpp>
#include <gpsd_client/gpsd_raw_message.hpp>

namespace gpsd_client
{

/// Converts a gpsd report into the raw message for this build's libgps API.
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

private:
  /// Number of skyview entries that are actually populated.
  ///
  /// gps_data_t::satellites_visible is a plain int that a partially-filled or
  /// stale report can leave negative or larger than the array, so it is
  /// clamped to [0, MAXCHANNELS] rather than trusted. Publishing the whole
  /// fixed array instead would emit MAXCHANNELS (140 or 184, depending on the
  /// gpsd) entries of uninitialised satellites.
  static std::size_t skyviewCount(const gps_data_t& data);

  ParserContext context_;
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_RAW_PARSER_HPP_
