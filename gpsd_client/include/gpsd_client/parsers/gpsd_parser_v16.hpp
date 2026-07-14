#ifndef GPSD_CLIENT__PARSERS__GPSD_PARSER_V16_HPP_
#define GPSD_CLIENT__PARSERS__GPSD_PARSER_V16_HPP_

#include <gps.h>

// Supports gpsd APIs 10 through 16. Per project convention, parsers are named
// after the highest API version they support: when a newer gpsd API is
// verified compatible, extend this guard and rename the class accordingly.
#if GPSD_API_MAJOR_VERSION >= 10

#include <gpsd_client/parsers/gpsd_parser_base.hpp>

namespace gpsd_client
{

/// Parser for gpsd API versions 10-16 (gpsd 3.21 - 3.27).
class GpsdParserV16 : public GpsdParserBase
{
public:
  using GpsdParserBase::GpsdParserBase;

protected:
  [[nodiscard]] int getFixStatus(const gps_data_t& data) const override
  {
    return data.fix.status;
  }
};

}  // namespace gpsd_client

#endif  // GPSD_API_MAJOR_VERSION >= 10

#endif  // GPSD_CLIENT__PARSERS__GPSD_PARSER_V16_HPP_
