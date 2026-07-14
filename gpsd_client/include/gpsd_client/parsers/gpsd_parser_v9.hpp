#ifndef GPSD_CLIENT__PARSERS__GPSD_PARSER_V9_HPP_
#define GPSD_CLIENT__PARSERS__GPSD_PARSER_V9_HPP_

#include <gps.h>

// This parser only compiles against gpsd API 9: it reads the fix status from
// gps_data_t::status, which was removed (moved into gps_fix_t) in API 10.
#if GPSD_API_MAJOR_VERSION == 9

#include <gpsd_client/parsers/gpsd_parser_base.hpp>

namespace gpsd_client
{

/// Parser for gpsd API version 9 (gpsd 3.20).
class GpsdParserV9 : public GpsdParserBase
{
public:
  using GpsdParserBase::GpsdParserBase;

protected:
  [[nodiscard]] int getFixStatus(const gps_data_t& data) const override
  {
    return data.status;
  }
};

}  // namespace gpsd_client

#endif  // GPSD_API_MAJOR_VERSION == 9

#endif  // GPSD_CLIENT__PARSERS__GPSD_PARSER_V9_HPP_
