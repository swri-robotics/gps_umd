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

#include <gpsd_client/gpsd_parser_factory.hpp>

#include <gpsd_client/parsers/gpsd_parser_v9.hpp>
#include <gpsd_client/parsers/gpsd_parser_v16.hpp>

namespace gpsd_client
{

std::unique_ptr<GpsdParser> GpsdParserFactory::create(const ParserContext & context)
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

std::unique_ptr<GpsdRawParser> GpsdParserFactory::createRaw(
  const ParserContext & context)
{
  // No ladder here on purpose: gpsd_raw_message.hpp has already resolved
  // GpsdRawMsg and pulled in the matching generated fill() for this build,
  // including the API < 9 error and the newer-than-tested fallback.
  return std::make_unique<GpsdRawParser>(context);
}

}  // namespace gpsd_client
