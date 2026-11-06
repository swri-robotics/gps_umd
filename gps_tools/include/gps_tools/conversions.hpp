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

#ifndef GPS_TOOLS__CONVERSIONS_HPP_
#define GPS_TOOLS__CONVERSIONS_HPP_

/**
 * @file
 * Conversions between WGS84 latitude/longitude and UTM coordinates.
 *
 * The projection is GeographicLib's transverse Mercator, accurate to a few
 * nanometers. These functions are compiled into the gps_tools_conversions
 * library, so link it (it is in ${gps_tools_LIBRARIES}).
 */

// <cstdio> and <cstdlib> are not used here; they are kept because the old
// header-only conversions.h provided them, and callers may rely on that.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace gps_tools
{

// These constants are not used by the conversions below, which take their
// ellipsoid from GeographicLib. They stay for code that already uses them.
const double RADIANS_PER_DEGREE = M_PI / 180.0;
const double DEGREES_PER_RADIAN = 180.0 / M_PI;

// WGS84 ellipsoid
const double WGS84_A = 6378137.0;         // semi-major axis (m)
const double WGS84_B = 6356752.31424518;  // semi-minor axis (m)
const double WGS84_F = 0.0033528107;      // flattening
const double WGS84_E = 0.0818191908;      // first eccentricity
const double WGS84_EP = 0.0820944379;     // second eccentricity

// UTM projection
const double UTM_K0 = 0.9996;          // scale factor on the central meridian
const double UTM_FE = 500000.0;        // false easting (m)
const double UTM_FN_N = 0.0;           // false northing, northern hemisphere (m)
const double UTM_FN_S = 10000000.0;    // false northing, southern hemisphere (m)
const double UTM_E2 = (WGS84_E * WGS84_E);       // e^2
const double UTM_E4 = (UTM_E2 * UTM_E2);         // e^4
const double UTM_E6 = (UTM_E4 * UTM_E2);         // e^6
const double UTM_EP2 = (UTM_E2 / (1 - UTM_E2));  // e'^2

/**
 * Project a latitude and longitude (degrees) into UTM easting @p x and
 * northing @p y (meters).
 *
 * The zone comes from the longitude alone, in 6 degree strips, without the
 * Norway and Svalbard exceptions LLtoUTM() applies. The equator is in the
 * northern hemisphere. A non-finite latitude or longitude gives NaN.
 */
void UTM(double lat, double lon, double * x, double * y);

/**
 * The UTM latitude band letter for @p Lat (degrees), 'C' through 'X'.
 *
 * Returns 'Z' if the latitude is outside the UTM limits of 80S to 84N, or is
 * NaN.
 */
char UTMLetterDesignator(double Lat);

/**
 * Convert a latitude and longitude (degrees, north and east positive) to UTM.
 *
 * Longitude is wrapped into [-180, 180) before the zone is chosen, and the
 * Norway and Svalbard zones are applied. @p UTMZone receives the zone number
 * and band letter, such as "14R"; the band is 'Z' outside 80S to 84N, where
 * the coordinates are still projected in the nearest zone. It must hold at
 * least 13 characters.
 *
 * A latitude outside [-90, 90] gives NaN coordinates. A non-finite longitude
 * gives NaN coordinates and zone number 0.
 */
void LLtoUTM(
  const double Lat, const double Long,
  double & UTMNorthing, double & UTMEasting,
  char * UTMZone);

/// LLtoUTM() returning the zone as a std::string.
void LLtoUTM(
  const double Lat, const double Long,
  double & UTMNorthing, double & UTMEasting,
  std::string & UTMZone);

/**
 * Convert UTM coordinates in @p UTMZone, such as "14R", to latitude and
 * longitude (degrees).
 *
 * A band letter before 'N', or no letter at all, means the southern
 * hemisphere. The longitude is not wrapped: a point past the antimeridian
 * from zone 60 comes back above 180 degrees.
 */
void UTMtoLL(
  const double UTMNorthing, const double UTMEasting,
  const char * UTMZone, double & Lat, double & Long);

/// UTMtoLL() taking the zone as a std::string.
void UTMtoLL(
  const double UTMNorthing, const double UTMEasting,
  std::string UTMZone, double & Lat, double & Long);

}  // namespace gps_tools

#endif  // GPS_TOOLS__CONVERSIONS_HPP_
