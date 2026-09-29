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

#ifndef GPSD_CLIENT__GPS_HPP_
#define GPSD_CLIENT__GPS_HPP_

/* The only place in gpsd_client that includes <gps.h>. Include this instead.
 *
 * gps.h defines its fix statuses as macros, and some share their names with
 * constants in the ROS messages: STATUS_RTK_FIX in every GPSd, and
 * STATUS_NO_FIX, STATUS_FIX and STATUS_DGPS_FIX before GPSd 3.23. A macro
 * cannot be put in a namespace, so any gps_msgs::msg::GPSStatus::STATUS_FIX
 * or sensor_msgs::msg::NavSatStatus::STATUS_FIX parsed after gps.h stopped
 * compiling.
 *
 * This header copies each of those macros' values into gpsd_client::gps_h,
 * under the same name, and then undefines the macro. Nothing gps.h declares
 * depends on them at runtime, and gps.h's include guard keeps a later
 * #include <gps.h> from bringing them back. Include order then no longer
 * matters. test_gps_h checks that every message constant compiles after this
 * header, and test_gps_h_includes that nothing else includes gps.h.
 *
 * Each value copied has a GPSD_CLIENT_HAS_<NAME> flag, for code that has to
 * ask whether this gps.h defined it, as the generated fill headers do. Keep
 * CAPTURED_MACROS in tools/generate_raw_msgs.py in step with the list below.
 */

#include <gps.h>

#if GPSD_API_MAJOR_VERSION < 9
// cppcheck does not read gps.h, so it takes the version as 0.
// cppcheck-suppress preprocessorErrorDirective
#error "gpsd_client requires GPSd API version >= 9 (GPSd >= 3.20)"
#endif

namespace gpsd_client
{
namespace gps_h
{
/* Fix statuses under one name across GPSd's renames, for the code that maps
 * them: GPSd 3.23 renamed STATUS_FIX to STATUS_GPS, and 3.25 renamed
 * STATUS_DGPS_FIX to STATUS_DGPS. The values did not change. Read before the
 * macros are undefined below.
 */
#ifdef STATUS_GPS
inline constexpr int kStatusGps = STATUS_GPS;
#else
inline constexpr int kStatusGps = STATUS_FIX;
#endif
#ifdef STATUS_DGPS
inline constexpr int kStatusDgps = STATUS_DGPS;
#else
inline constexpr int kStatusDgps = STATUS_DGPS_FIX;
#endif
inline constexpr int kStatusRtkFix = STATUS_RTK_FIX;
inline constexpr int kStatusRtkFloat = STATUS_RTK_FLT;

namespace detail
{
#ifdef STATUS_NO_FIX
inline constexpr int status_no_fix = STATUS_NO_FIX;
#endif
#ifdef STATUS_FIX
inline constexpr int status_fix = STATUS_FIX;
#endif
#ifdef STATUS_DGPS_FIX
inline constexpr int status_dgps_fix = STATUS_DGPS_FIX;
#endif
inline constexpr int status_rtk_fix = STATUS_RTK_FIX;
}  // namespace detail
}  // namespace gps_h
}  // namespace gpsd_client

#ifdef STATUS_NO_FIX
#undef STATUS_NO_FIX
#define GPSD_CLIENT_HAS_STATUS_NO_FIX
#endif
#ifdef STATUS_FIX
#undef STATUS_FIX
#define GPSD_CLIENT_HAS_STATUS_FIX
#endif
#ifdef STATUS_DGPS_FIX
#undef STATUS_DGPS_FIX
#define GPSD_CLIENT_HAS_STATUS_DGPS_FIX
#endif
#undef STATUS_RTK_FIX
#define GPSD_CLIENT_HAS_STATUS_RTK_FIX

namespace gpsd_client
{
namespace gps_h
{
// gps.h's values for the macros undefined above, under their own names.
#ifdef GPSD_CLIENT_HAS_STATUS_NO_FIX
inline constexpr int STATUS_NO_FIX = detail::status_no_fix;
#endif
#ifdef GPSD_CLIENT_HAS_STATUS_FIX
inline constexpr int STATUS_FIX = detail::status_fix;
#endif
#ifdef GPSD_CLIENT_HAS_STATUS_DGPS_FIX
inline constexpr int STATUS_DGPS_FIX = detail::status_dgps_fix;
#endif
inline constexpr int STATUS_RTK_FIX = detail::status_rtk_fix;
}  // namespace gps_h
}  // namespace gpsd_client

#endif  // GPSD_CLIENT__GPS_HPP_
