#ifndef GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_
#define GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_

#include <memory>

#include <gpsd_client/gpsd_parser.hpp>

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
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_
