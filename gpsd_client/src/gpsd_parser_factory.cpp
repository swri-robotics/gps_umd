#include <gpsd_client/gpsd_parser_factory.hpp>

#include <gpsd_client/parsers/gpsd_parser_v9.hpp>
#include <gpsd_client/parsers/gpsd_parser_v16.hpp>

namespace gpsd_client
{

std::unique_ptr<GpsdParser> GpsdParserFactory::create(const ParserContext& context)
{
  // The one and only version-selection ladder. See the class comment in
  // gpsd_parser_factory.hpp for why this is compile-time rather than runtime.
#if GPSD_API_MAJOR_VERSION == 9
  return std::make_unique<GpsdParserV9>(context);
#elif GPSD_API_MAJOR_VERSION <= 16
  return std::make_unique<GpsdParserV16>(context);
#else
  // Newer than tested: warn at compile time and use the newest parser. Once
  // verified compatible, extend the guard in gpsd_parser_v16.hpp and rename
  // the parser after the new highest supported version.
#warning "Untested GPSD_API_MAJOR_VERSION > 16; falling back to GpsdParserV16"
  return std::make_unique<GpsdParserV16>(context);
#endif
}

}  // namespace gpsd_client
