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

#ifndef GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_
#define GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_

#include <memory>

#include <gpsd_client/gpsd_parser.hpp>
#include <gpsd_client/gpsd_raw_parser.hpp>

namespace gpsd_client
{

/// Produces the GpsdParser matching the GPSd API this package was built
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
  static std::unique_ptr<GpsdParser> create(const ParserContext & context);

  /// Produces the raw parser for this build's libgps API.
  ///
  /// Kept here so callers have one place to ask for a parser, but note the
  /// version selection itself lives in gpsd_raw_message.hpp rather than in
  /// this file: the raw message *type* varies with the API pair, so the choice
  /// has to be made where the type alias is declared. There is still exactly
  /// one ladder per output -- this one for GPSFix/NavSatFix, that one for raw.
  static std::unique_ptr<GpsdRawParser> createRaw(const ParserContext & context);
};

}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPSD_PARSER_FACTORY_HPP_
