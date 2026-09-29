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

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "gps_tools/conversions.h"

namespace
{

struct Reference
{
  const char * name;
  double lat;
  double lon;
  int zone;
  double easting;
  double northing;
};

// Produced by GeographicLib::UTMUPS::Forward() (GeographicLib 2.7), which is
// accurate to a few nanometers, including its Norway and Svalbard zones.
const std::vector<Reference> kReferences = {
  {"San Antonio", 29.4241, -98.4936, 14, 549120.9765, 3255080.3933},
  {"London", 51.5074, -0.1278, 30, 699316.2343, 5710163.7581},
  {"Sydney", -33.8688, 151.2093, 56, 334368.6336, 6250948.3454},
  {"Bergen", 60.3913, 5.3221, 32, 297353.9327, 6700648.3452},
  {"Longyearbyen", 78.2232, 15.6267, 33, 514278.7151, 8683355.4695},
  {"Quito", -0.1807, -78.4678, 17, 781861.4575, 9980007.5669},
  {"Near the west edge of zone 11", 45.0000, -119.9000, 11, 271435.5141, 4987042.3066},
  {"Near the east edge of zone 11", 45.0000, -114.1000, 11, 728564.4859, 4987042.3066},
  {"Far south", -79.9000, 30.0000, 36, 441292.5527, 1128062.1714},
  {"Far north", 83.9000, -40.0000, 24, 488136.7308, 9317033.0971},
  {"West of the antimeridian", 10.0000, 179.9000, 60, 817955.4277, 1106810.6571},
  {"East of the antimeridian", 10.0000, -179.9000, 1, 182044.5723, 1106810.6571},
};

// The series LLtoUTM() uses stays well inside a millimeter at these points.
constexpr double kPositionTolerance = 1e-3;
constexpr double kRoundTripTolerance = 0.01;
// About a millimeter of latitude.
constexpr double kAngleTolerance = 1e-8;
constexpr double kMetersPerDegree = 111320.0;

struct Utm
{
  double northing;
  double easting;
  std::string zone;
};

Utm toUtm(double lat, double lon)
{
  Utm utm;
  gps_tools::LLtoUTM(lat, lon, utm.northing, utm.easting, utm.zone);
  return utm;
}

int zoneNumber(const std::string & zone)
{
  return std::stoi(zone);
}

}  // namespace

TEST(LLtoUTM, MatchesGeographicLib)
{
  for (const Reference & ref : kReferences) {
    SCOPED_TRACE(ref.name);
    const Utm utm = toUtm(ref.lat, ref.lon);
    EXPECT_EQ(ref.zone, zoneNumber(utm.zone));
    EXPECT_NEAR(ref.easting, utm.easting, kPositionTolerance);
    EXPECT_NEAR(ref.northing, utm.northing, kPositionTolerance);
  }
}

TEST(LLtoUTM, ZoneLetterFollowsLatitudeBands)
{
  EXPECT_EQ("14R", toUtm(29.4241, -98.4936).zone);
  EXPECT_EQ("56H", toUtm(-33.8688, 151.2093).zone);
  // The equator belongs to band N, and anything south of it to M.
  EXPECT_EQ("31N", toUtm(0.0, 3.0).zone);
  EXPECT_EQ("31M", toUtm(-1e-9, 3.0).zone);
  // Band X is 12 degrees tall and runs all the way to 84N.
  EXPECT_EQ("31X", toUtm(84.0, 3.0).zone);
}

TEST(UTMLetterDesignator, MarksLatitudesOutsideUtmWithZ)
{
  EXPECT_EQ('X', gps_tools::UTMLetterDesignator(84.0));
  EXPECT_EQ('Z', gps_tools::UTMLetterDesignator(84.5));
  EXPECT_EQ('C', gps_tools::UTMLetterDesignator(-80.0));
  EXPECT_EQ('Z', gps_tools::UTMLetterDesignator(-80.5));
  EXPECT_EQ('X', gps_tools::UTMLetterDesignator(72.0));
  EXPECT_EQ('W', gps_tools::UTMLetterDesignator(71.999));
}

TEST(LLtoUTM, UsesTheNorwayAndSvalbardZones)
{
  // Southwest Norway widens zone 32 west to 3E.
  EXPECT_EQ("32V", toUtm(56.0, 3.0).zone);
  EXPECT_EQ("32V", toUtm(63.999, 11.999).zone);
  EXPECT_EQ("31W", toUtm(64.0, 5.0).zone);
  // Svalbard uses only the odd zones 31 to 37.
  EXPECT_EQ("31X", toUtm(72.0, 8.999).zone);
  EXPECT_EQ("33X", toUtm(72.0, 9.0).zone);
  EXPECT_EQ("35X", toUtm(78.0, 21.0).zone);
  EXPECT_EQ("37X", toUtm(83.999, 41.999).zone);
}

TEST(LLtoUTM, WrapsLongitudeIntoRange)
{
  // Each pair names the same meridian.
  const std::vector<std::pair<double, double>> equivalent = {
    {180.0, -180.0},
    {181.0, -179.0},
    {540.5, -179.5},
    {-180.5, 179.5},
    {-190.0, 170.0},
    {-359.0, 1.0},
  };
  for (const auto & [lon, wrapped] : equivalent) {
    SCOPED_TRACE(lon);
    const Utm utm = toUtm(10.0, lon);
    const Utm expected = toUtm(10.0, wrapped);
    EXPECT_EQ(expected.zone, utm.zone);
    EXPECT_NEAR(expected.easting, utm.easting, kPositionTolerance);
    EXPECT_NEAR(expected.northing, utm.northing, kPositionTolerance);
  }
}

TEST(LLtoUTM, CharAndStringOverloadsAgree)
{
  double northing = 0.0;
  double easting = 0.0;
  char zone[13] = {0};
  gps_tools::LLtoUTM(-33.8688, 151.2093, northing, easting, zone);

  const Utm utm = toUtm(-33.8688, 151.2093);
  EXPECT_EQ(utm.zone, std::string(zone));
  EXPECT_EQ(utm.easting, easting);
  EXPECT_EQ(utm.northing, northing);
}

TEST(UTMtoLL, InvertsLLtoUTMEverywhere)
{
  for (double lat = -79.5; lat <= 83.5; lat += 3.25) {
    for (double lon = -179.5; lon <= 179.5; lon += 7.75) {
      SCOPED_TRACE(std::to_string(lat) + ", " + std::to_string(lon));
      const Utm utm = toUtm(lat, lon);
      double lat_back = 0.0;
      double lon_back = 0.0;
      gps_tools::UTMtoLL(utm.northing, utm.easting, utm.zone, lat_back, lon_back);
      // Compare on the ground, since a degree of longitude shrinks toward
      // the poles. The inverse series is the less accurate one, and it drifts
      // furthest in the wide Svalbard zones: about 9 mm at 73N 22E, 5 degrees
      // from the central meridian of zone 35X.
      const double north_error = (lat_back - lat) * kMetersPerDegree;
      const double east_error =
        (lon_back - lon) * kMetersPerDegree * std::cos(lat * M_PI / 180.0);
      EXPECT_LT(std::hypot(north_error, east_error), kRoundTripTolerance);
    }
  }
}

TEST(UTMtoLL, TakesTheHemisphereFromTheZoneLetter)
{
  const Utm north = toUtm(1.0, 3.0);
  const Utm south = toUtm(-1.0, 3.0);
  double lat = 0.0;
  double lon = 0.0;

  gps_tools::UTMtoLL(north.northing, north.easting, "31N", lat, lon);
  EXPECT_NEAR(1.0, lat, kAngleTolerance);
  gps_tools::UTMtoLL(south.northing, south.easting, "31M", lat, lon);
  EXPECT_NEAR(-1.0, lat, kAngleTolerance);
}

TEST(UTM, AgreesWithLLtoUTM)
{
  // UTM() picks the zone from the longitude alone, so keep these points away
  // from the Norway and Svalbard zones.
  const std::vector<std::pair<double, double>> points = {
    {29.4241, -98.4936}, {-33.8688, 151.2093}, {45.0, -119.9}, {0.5, 0.5}, {-0.5, -0.5},
  };
  for (const auto & [lat, lon] : points) {
    SCOPED_TRACE(std::to_string(lat) + ", " + std::to_string(lon));
    double x = 0.0;
    double y = 0.0;
    gps_tools::UTM(lat, lon, &x, &y);
    const Utm utm = toUtm(lat, lon);
    EXPECT_NEAR(utm.easting, x, kPositionTolerance);
    EXPECT_NEAR(utm.northing, y, kPositionTolerance);
  }
}

TEST(UTM, PutsTheEquatorInTheNorthernHemisphere)
{
  double x = 0.0;
  double y = 0.0;
  gps_tools::UTM(0.0, 3.0, &x, &y);
  EXPECT_NEAR(0.0, y, kPositionTolerance);
  EXPECT_NEAR(toUtm(0.0, 3.0).northing, y, kPositionTolerance);
}
