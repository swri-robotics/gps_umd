#ifndef GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_
#define GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_

#include <memory>

#include <gpsd_client/gpsd_parser.hpp>
#include <gpsd_client/gpsd_raw_parser.hpp>

namespace gpsd_client
{

/// Produces the GpsdParser matching the gpsd API this package was built
/// against.
///
/// Note: only one libgps is present at build time and the layout of
/// gps_data_t is fixed by its header, so exactly one parser implementation
/// can ever be compiled into a given binary. The factory therefore selects
/// the parser with a compile-time ladder on GPSD_API_MAJOR_VERSION rather
/// than a runtime registry.
class GpsdParserFactory
{
public:
  static std::unique_ptr<GpsdParser> create(const ParserContext& context);

  /// Produces the raw parser for this build's libgps API.
  ///
  /// Kept here so callers have one place to ask for a parser, but note the
  /// version selection itself lives in gpsd_raw_message.hpp rather than in
  /// this file: the raw message *type* varies with the API pair, so the choice
  /// has to be made where the type alias is declared. There is still exactly
  /// one ladder per output -- this one for GPSFix/NavSatFix, that one for raw.
  static std::unique_ptr<GpsdRawParser> createRaw(const ParserContext& context);
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_
