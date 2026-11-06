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

#include "gps_tools/conversions.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <GeographicLib/TransverseMercator.hpp>

// The projection is GeographicLib::TransverseMercator::UTM() rather than
// GeographicLib::UTMUPS, which also picks the zone. UTMUPS throws outside the
// UTM latitudes, or switches to UPS there, and applies the Svalbard zones
// north of 84N. These functions have always projected such points in the
// nearest UTM zone, so the zone is chosen here and only the projection, the
// hard part, comes from GeographicLib.

namespace gps_tools
{

namespace
{

// Room for any "%d%c" zone designator, matching the buffer callers of the
// char * LLtoUTM() have always been told to pass.
constexpr std::size_t kZoneLength = 13;
// The zone number in a designator is written in decimal.
constexpr int kDecimalBase = 10;

// Angles, in degrees.
constexpr double kHalfTurn = 180.0;
constexpr double kFullTurn = 360.0;

// UTM zones are strips of longitude numbered eastward, starting from 1 at the
// antimeridian.
constexpr double kZoneWidth = 6.0;
constexpr double kHalfZoneWidth = kZoneWidth / 2.0;
constexpr double kFirstZoneWestEdge = -kHalfTurn;
constexpr int kFirstZone = 1;
// The zone number given to a longitude that has none, such as NaN.
constexpr int kNoZone = 0;

// Latitude bands are lettered northward from the southern limit of UTM. All
// are kBandHeight tall except the last, X, which reaches to the northern
// limit and so is half again as tall.
constexpr char kBands[] = "CDEFGHJKLMNPQRSTUVWX";
constexpr int kBandCount = sizeof(kBands) - 1;
constexpr double kBandHeight = 8.0;
constexpr double kSouthernLimit = -80.0;
constexpr double kNorthernLimit = 84.0;
// The band letter for a latitude outside the UTM limits.
constexpr char kNoBand = 'Z';
// The southernmost band north of the equator. Earlier letters are south.
constexpr char kFirstNorthernBand = 'N';

// Southwest Norway: zone 32 is widened west over band V of zone 31.
constexpr double kNorwaySouthEdge = 56.0;
constexpr double kNorwayNorthEdge = 64.0;
constexpr double kNorwayWestEdge = 3.0;
constexpr double kNorwayEastEdge = 12.0;
constexpr int kNorwayZone = 32;

// Svalbard: band X uses only the odd zones from kSvalbardWestZone to
// kSvalbardEastZone. Each even zone between them is split down its central
// meridian, and its odd neighbours each take half.
constexpr double kSvalbardSouthEdge = 72.0;
constexpr double kSvalbardNorthEdge = kNorthernLimit;
constexpr int kSvalbardWestZone = 31;
constexpr int kSvalbardEastZone = 37;

// The edges and center of a zone's standard strip of longitude. All are
// whole degrees, which doubles hold exactly.
double zoneWestEdge(int zone)
{
  return kFirstZoneWestEdge + (zone - kFirstZone) * kZoneWidth;
}

double zoneEastEdge(int zone)
{
  return zoneWestEdge(zone) + kZoneWidth;
}

double centralMeridian(int zone)
{
  return zoneWestEdge(zone) + kHalfZoneWidth;
}

// Wraps a longitude into [-180, 180).
double wrapLongitude(double lon)
{
  return (lon + kHalfTurn) - std::floor((lon + kHalfTurn) / kFullTurn) * kFullTurn - kHalfTurn;
}

// The UTM zone for a latitude and a longitude already in [-180, 180),
// including the Norway and Svalbard exceptions.
int zoneNumber(double lat, double lon)
{
  if (lat >= kSvalbardSouthEdge && lat < kSvalbardNorthEdge &&
    lon >= zoneWestEdge(kSvalbardWestZone))
  {
    for (int zone = kSvalbardWestZone; zone < kSvalbardEastZone; zone += 2) {
      // The odd zone reaches to the middle of the even zone east of it.
      if (lon < centralMeridian(zone + 1)) {
        return zone;
      }
    }
    if (lon < zoneEastEdge(kSvalbardEastZone)) {
      return kSvalbardEastZone;
    }
  }

  if (lat >= kNorwaySouthEdge && lat < kNorwayNorthEdge &&
    lon >= kNorwayWestEdge && lon < kNorwayEastEdge)
  {
    return kNorwayZone;
  }

  return static_cast<int>((lon - kFirstZoneWestEdge) / kZoneWidth) + kFirstZone;
}

// Projects into the UTM zone centered on lon0. The equator is in the northern
// hemisphere.
void project(double lat, double lon, double lon0, double & easting, double & northing)
{
  double x = 0.0;
  double y = 0.0;
  GeographicLib::TransverseMercator::UTM().Forward(lon0, lat, lon, x, y);
  easting = x + UTM_FE;
  northing = y + (lat < 0.0 ? UTM_FN_S : UTM_FN_N);
}

}  // namespace

void UTM(double lat, double lon, double * x, double * y)
{
  if (!std::isfinite(lat) || !std::isfinite(lon)) {
    *x = NAN;
    *y = NAN;
    return;
  }

  // The central meridian of the zone-wide strip holding lon. On a negative
  // multiple of the zone width this takes the strip to the west, as UTM()
  // always has.
  const double whole = std::trunc(lon);
  const double cm =
    whole - std::fmod(whole, kZoneWidth) + (lon >= 0.0 ? kHalfZoneWidth : -kHalfZoneWidth);

  project(lat, lon, cm, *x, *y);
}

char UTMLetterDesignator(double Lat)
{
  // Comparing against each band's southern edge, rather than dividing, keeps
  // the edges exact. The negated test also sends NaN to kNoBand.
  if (!(Lat >= kSouthernLimit && Lat <= kNorthernLimit)) {
    return kNoBand;
  }
  int band = kBandCount - 1;
  while (band > 0 && Lat < kSouthernLimit + kBandHeight * band) {
    --band;
  }
  return kBands[band];
}

void LLtoUTM(
  const double Lat, const double Long,
  double & UTMNorthing, double & UTMEasting,
  char * UTMZone)
{
  const double lon = wrapLongitude(Long);
  const int zone = std::isfinite(lon) ? zoneNumber(Lat, lon) : kNoZone;

  snprintf(UTMZone, kZoneLength, "%d%c", zone, UTMLetterDesignator(Lat));

  if (zone == kNoZone) {
    UTMNorthing = NAN;
    UTMEasting = NAN;
    return;
  }
  project(Lat, lon, centralMeridian(zone), UTMEasting, UTMNorthing);
}

void LLtoUTM(
  const double Lat, const double Long,
  double & UTMNorthing, double & UTMEasting,
  std::string & UTMZone)
{
  char zone[kZoneLength] = {0};
  LLtoUTM(Lat, Long, UTMNorthing, UTMEasting, zone);
  UTMZone = zone;
}

void UTMtoLL(
  const double UTMNorthing, const double UTMEasting,
  const char * UTMZone, double & Lat, double & Long)
{
  char * letter = nullptr;
  const int zone = static_cast<int>(std::strtol(UTMZone, &letter, kDecimalBase));
  // A missing letter reads as the terminating '\0', and so as south too.
  const bool south = *letter < kFirstNorthernBand;
  const double lon0 = centralMeridian(zone);

  double lon = 0.0;
  GeographicLib::TransverseMercator::UTM().Reverse(
    lon0, UTMEasting - UTM_FE, UTMNorthing - (south ? UTM_FN_S : UTM_FN_N), Lat, lon);

  // GeographicLib wraps the longitude into [-180, 180]. Report it relative to
  // the zone's central meridian instead, as UTMtoLL() always has, so a point
  // just east of zone 60 comes back above 180 rather than near -180.
  Long = lon0 + std::remainder(lon - lon0, kFullTurn);
}

void UTMtoLL(
  const double UTMNorthing, const double UTMEasting,
  std::string UTMZone, double & Lat, double & Long)
{
  UTMtoLL(UTMNorthing, UTMEasting, UTMZone.c_str(), Lat, Long);
}

}  // namespace gps_tools
