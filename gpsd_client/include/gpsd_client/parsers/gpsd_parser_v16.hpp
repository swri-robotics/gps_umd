// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef GPSD_CLIENT__PARSERS__GPSD_PARSER_V16_HPP_
#define GPSD_CLIENT__PARSERS__GPSD_PARSER_V16_HPP_

#include <gpsd_client/gps.hpp>

// Supports GPSd APIs 10 through 16. Per project convention, parsers are named
// after the highest API version they support: when a newer GPSd API is
// verified compatible, extend this guard and rename the class accordingly.
#if GPSD_API_MAJOR_VERSION >= 10

#include <gpsd_client/parsers/gpsd_parser_base.hpp>

namespace gpsd_client
{

/// Parser for GPSd API versions 10-16 (GPSd 3.21 - 3.27).
class GpsdParserV16 : public GpsdParserBase
{
public:
  using GpsdParserBase::GpsdParserBase;

protected:
  [[nodiscard]] int getFixStatus(const gps_data_t & data) const override
  {
    return data.fix.status;
  }
};

}  // namespace gpsd_client

#endif  // GPSD_API_MAJOR_VERSION >= 10

#endif  // GPSD_CLIENT__PARSERS__GPSD_PARSER_V16_HPP_
