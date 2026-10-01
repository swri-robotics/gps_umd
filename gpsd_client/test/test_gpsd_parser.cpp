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
#include <cstdint>
#include <memory>
#include <vector>

#include <gpsd_client/gpsd_parser_factory.hpp>

namespace
{

// The tests, like the parsers, can only compile against the single installed
// libgps header, so they exercise whichever parser the factory selects. Where
// the fix status lives depends on that header, and is isolated here.
//
// HAVE_GPS_FIX_STATUS comes from CheckStructHasMember in CMakeLists.txt and
// asks the header directly rather than inferring from the version. GPSd moved
// the status from gps_data_t to gps_fix_t on the API 10 bump commit itself, so
// a version comparison happens to work here -- but one API pair spans a range
// of header states, so version arithmetic does not answer "has this member" in
// general. See docs/gpsd-quirks.md.
void setFixStatus(gps_data_t & data, int status)
{
#ifdef HAVE_GPS_FIX_STATUS
  data.fix.status = status;
#else
  data.status = status;
#endif
}

using gpsd_client::gps_h::kStatusDgps;
using gpsd_client::gps_h::kStatusGps;

/// The variance published for a GPSd uncertainty, with GPSd's 95% figure
/// taken as 1.96 standard deviations unless told otherwise.
double expectedVariance(double uncertainty, double to_sigma = 1.96)
{
  const double sigma = uncertainty / to_sigma;
  return sigma * sigma;
}

gpsd_client::ParserContext makeContext()
{
  gpsd_client::ParserContext context;
  context.frame_id = "gps";
  context.use_gps_time = false;
  context.check_fix_by_variance = false;
  context.override_augmentation_source = false;
  return context;
}

// A plausible online 3D fix with two used GPS satellites.
gps_data_t makeThreeDFix()
{
  gps_data_t data{};

  data.online.tv_sec = 100;
  data.fix.mode = MODE_3D;
  setFixStatus(data, kStatusGps);

  data.fix.time.tv_sec = 1700000000;
  data.fix.time.tv_nsec = 500000000;

  data.fix.latitude = 29.44;
  data.fix.longitude = -98.61;
  // As GPSd fills them: the deprecated member carries sea level, not the
  // ellipsoid height NavSatFix wants.
  data.fix.altHAE = 250.0;
  data.fix.altMSL = 280.0;
  data.fix.altitude = 280.0;
  data.fix.track = 90.0;
  data.fix.speed = 2.5;
  data.fix.climb = 0.25;

  data.fix.epx = 1.5;
  data.fix.epy = 2.5;
  data.fix.epv = 3.5;
  data.fix.epd = 0.5;
  data.fix.eps = 0.75;
  data.fix.epc = 1.25;
  data.fix.ept = 0.005;
  data.fix.eph = 4.5;

  data.dop.pdop = 1.1;
  data.dop.hdop = 1.2;
  data.dop.vdop = 1.3;
  data.dop.tdop = 1.4;
  data.dop.gdop = 1.5;

  data.satellites_used = 2;
  data.satellites_visible = 3;
  for (int i = 0; i < 3; ++i) {
    data.skyview[i].PRN = 10 + i;
    data.skyview[i].elevation = 30 + i;
    data.skyview[i].azimuth = 100 + i;
    data.skyview[i].ss = 40 + i;
    data.skyview[i].gnssid = GNSSID_GPS;
    data.skyview[i].used = i < 2;
  }

  return data;
}

std::unique_ptr<gpsd_client::GpsdParser> makeParser(
  const gpsd_client::ParserContext & context = makeContext())
{
  return gpsd_client::GpsdParserFactory::create(context);
}

}  // namespace

TEST(GpsdParser, ReportsOnline)
{
  auto parser = makeParser();

  gps_data_t data{};
  EXPECT_FALSE(parser->isOnline(data));

  data.online.tv_sec = 100;
  EXPECT_TRUE(parser->isOnline(data));
}

TEST(GpsdParser, ThreeDFixPopulatesGpsFix)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));

  EXPECT_EQ(fix.header.frame_id, "gps");
  EXPECT_EQ(fix.header.stamp.sec, 42);

  EXPECT_DOUBLE_EQ(fix.latitude, 29.44);
  EXPECT_DOUBLE_EQ(fix.longitude, -98.61);
  EXPECT_DOUBLE_EQ(fix.altitude, 250.0);
  EXPECT_DOUBLE_EQ(fix.track, 90.0);
  EXPECT_DOUBLE_EQ(fix.speed, 2.5);
  EXPECT_DOUBLE_EQ(fix.climb, 0.25);
  EXPECT_DOUBLE_EQ(fix.time, 1700000000.5);

  EXPECT_DOUBLE_EQ(fix.pdop, 1.1);
  EXPECT_DOUBLE_EQ(fix.hdop, 1.2);
  EXPECT_DOUBLE_EQ(fix.vdop, 1.3);
  EXPECT_DOUBLE_EQ(fix.tdop, 1.4);
  EXPECT_DOUBLE_EQ(fix.gdop, 1.5);

  EXPECT_DOUBLE_EQ(fix.err, 4.5);
  EXPECT_DOUBLE_EQ(fix.err_vert, 3.5);
  EXPECT_DOUBLE_EQ(fix.err_track, 0.5);
  EXPECT_DOUBLE_EQ(fix.err_speed, 0.75);
  EXPECT_DOUBLE_EQ(fix.err_climb, 1.25);
  EXPECT_DOUBLE_EQ(fix.err_time, 0.005);

  EXPECT_DOUBLE_EQ(fix.position_covariance[0], expectedVariance(1.5));
  EXPECT_DOUBLE_EQ(fix.position_covariance[4], expectedVariance(2.5));
  EXPECT_DOUBLE_EQ(fix.position_covariance[8], expectedVariance(3.5));
  EXPECT_EQ(
    fix.position_covariance_type, gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);

  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_FIX);
  EXPECT_EQ(fix.status.satellites_used, 2);
  ASSERT_EQ(fix.status.satellite_used_prn.size(), 2u);
  EXPECT_EQ(fix.status.satellite_used_prn[0], 10);
  EXPECT_EQ(fix.status.satellite_used_prn[1], 11);
  EXPECT_EQ(fix.status.satellites_visible, 3);
  ASSERT_EQ(fix.status.satellite_visible_prn.size(), 3u);
  EXPECT_EQ(fix.status.satellite_visible_prn[2], 12);
  EXPECT_EQ(fix.status.satellite_visible_z[2], 32);
  EXPECT_EQ(fix.status.satellite_visible_azimuth[2], 102);
  EXPECT_EQ(fix.status.satellite_visible_snr[2], 42);
}

TEST(GpsdParser, ThreeDFixPopulatesNavSatFix)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();

  auto fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));

  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->header.frame_id, "gps");
  EXPECT_EQ(fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_EQ(fix->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
  EXPECT_DOUBLE_EQ(fix->latitude, 29.44);
  EXPECT_DOUBLE_EQ(fix->longitude, -98.61);
  EXPECT_DOUBLE_EQ(fix->altitude, 250.0);
  // GPSd's uncertainties are meters; the covariance holds variances, m².
  EXPECT_DOUBLE_EQ(fix->position_covariance[0], expectedVariance(1.5));
  EXPECT_DOUBLE_EQ(fix->position_covariance[4], expectedVariance(2.5));
  EXPECT_DOUBLE_EQ(fix->position_covariance[8], expectedVariance(3.5));
  // The off-diagonal terms are unknown to GPSd and stay zero.
  EXPECT_DOUBLE_EQ(fix->position_covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(fix->position_covariance[5], 0.0);
  EXPECT_EQ(
    fix->position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
}

TEST(GpsdParser, TwoDFixHasNanAltitude)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  data.fix.mode = MODE_2D;

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));

  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_FIX);
  EXPECT_DOUBLE_EQ(fix.latitude, 29.44);
  EXPECT_TRUE(std::isnan(fix.altitude));

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_DOUBLE_EQ(navsat_fix->latitude, 29.44);
  EXPECT_TRUE(std::isnan(navsat_fix->altitude));
}

TEST(GpsdParser, AltitudeIsAboveTheEllipsoid)
{
  // NavSatFix defines altitude as height above the WGS 84 ellipsoid. GPSd's
  // deprecated altitude member is height above sea level whenever it has one.
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  data.fix.altHAE = -44.3;
  data.fix.altMSL = -30.4;
  data.fix.altitude = -30.4;

  EXPECT_DOUBLE_EQ(parser->parseGpsFix(data, rclcpp::Time(42, 0)).altitude, -44.3);
  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_DOUBLE_EQ(navsat_fix->altitude, -44.3);
}

TEST(GpsdParser, NoFixSetsNoFixStatus)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  data.fix.mode = MODE_NO_FIX;

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_NO_FIX);
  // Without a fix every measurement is unknown, not 0: 0,0 is a real place.
  const std::vector<double> measurements = {
    fix.time, fix.latitude, fix.longitude, fix.altitude, fix.track, fix.speed,
    fix.climb, fix.pdop, fix.hdop, fix.vdop, fix.tdop, fix.gdop, fix.err,
    fix.err_vert, fix.err_track, fix.err_speed, fix.err_climb, fix.err_time};
  for (const double value : measurements) {
    EXPECT_TRUE(std::isnan(value));
  }
  // The skyview is still valid, and there is no covariance to report.
  EXPECT_EQ(fix.status.satellites_visible, 3);
  EXPECT_EQ(fix.position_covariance_type, gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN);

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());  // variance check is off
  EXPECT_EQ(navsat_fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}

TEST(GpsdParser, InvalidVarianceRejectsNavSatFixOnly)
{
  auto context = makeContext();
  context.check_fix_by_variance = true;
  auto parser = makeParser(context);

  gps_data_t data = makeThreeDFix();
  data.fix.epx = std::nan("");

  // The NavSatFix is suppressed entirely ...
  EXPECT_FALSE(parser->parseNavSatFix(data, rclcpp::Time(42, 0)).has_value());

  // ... while the GPSFix is still produced, downgraded to NO_FIX.
  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_NO_FIX);
}

TEST(GpsdParser, NanVarianceMarksCovarianceUnknown)
{
  // With the variance check off, a fix with a NaN variance is still published,
  // so it must not claim to carry a known covariance.
  auto parser = makeParser();

  gps_data_t data = makeThreeDFix();
  data.fix.epv = std::nan("");

  auto fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));

  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(
    fix->position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
  // An unknown covariance is zero-filled; no NaN reaches subscribers.
  for (const double element : fix->position_covariance) {
    EXPECT_DOUBLE_EQ(element, 0.0);
  }

  // The same goes for GPSFix.
  gps_msgs::msg::GPSFix gps_fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(
    gps_fix.position_covariance_type, gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN);
  for (const double element : gps_fix.position_covariance) {
    EXPECT_DOUBLE_EQ(element, 0.0);
  }
}

TEST(GpsdParser, UncertaintyToSigmaScalesTheCovariance)
{
  // 1.0 treats GPSd's uncertainties as standard deviations.
  auto context = makeContext();
  context.uncertainty_to_sigma = 1.0;
  auto parser = makeParser(context);
  gps_data_t data = makeThreeDFix();

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_DOUBLE_EQ(navsat_fix->position_covariance[0], 2.25);
  EXPECT_DOUBLE_EQ(navsat_fix->position_covariance[4], 6.25);
  EXPECT_DOUBLE_EQ(navsat_fix->position_covariance[8], 12.25);

  gps_msgs::msg::GPSFix gps_fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_DOUBLE_EQ(gps_fix.position_covariance[0], 2.25);
  EXPECT_DOUBLE_EQ(gps_fix.position_covariance[4], 6.25);
  EXPECT_DOUBLE_EQ(gps_fix.position_covariance[8], 12.25);
}

TEST(GpsdParser, LegacyFixSemanticsKeepsTheOldNavSatFixCovariance)
{
  auto context = makeContext();
  context.legacy_fix_semantics = true;
  auto parser = makeParser(context);
  gps_data_t data = makeThreeDFix();

  // NavSatFix carries the uncertainties themselves, as it used to.
  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_DOUBLE_EQ(navsat_fix->position_covariance[0], 1.5);
  EXPECT_DOUBLE_EQ(navsat_fix->position_covariance[4], 2.5);
  EXPECT_DOUBLE_EQ(navsat_fix->position_covariance[8], 3.5);
  EXPECT_EQ(
    navsat_fix->position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);

  // GPSFix never had a covariance to be compatible with, so it gets variances.
  gps_msgs::msg::GPSFix gps_fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_DOUBLE_EQ(gps_fix.position_covariance[0], expectedVariance(1.5));
}

TEST(GpsdParser, ServiceBitmaskAndSbasStatus)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  setFixStatus(data, kStatusDgps);
  data.skyview[1].gnssid = GNSSID_GLO;
  data.skyview[2].gnssid = GNSSID_SBAS;
  data.skyview[2].used = true;
  data.satellites_used = 3;

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(
    navsat_fix->status.service,
    sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS);
  EXPECT_EQ(navsat_fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX);
}

TEST(GpsdParser, DgpsStatusWithoutSbas)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  setFixStatus(data, kStatusDgps);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX);

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX);
}

namespace
{
struct StatusCase
{
  const char * name;
  int gpsd_status;
  int16_t gps_status;
  int8_t navsat_status;
  uint16_t position_source;
  uint16_t motion_source;
};

void checkStatus(const gpsd_client::ParserContext & context, const StatusCase & c)
{
  SCOPED_TRACE(c.name);
  auto parser = makeParser(context);
  gps_data_t data = makeThreeDFix();
  setFixStatus(data, c.gpsd_status);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, c.gps_status);
  EXPECT_EQ(fix.status.position_source, c.position_source);
  EXPECT_EQ(fix.status.motion_source, c.motion_source);
  EXPECT_EQ(fix.status.orientation_source, c.motion_source);

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, c.navsat_status);
}
}  // namespace

TEST(GpsdParser, MapsEveryGpsdStatus)
{
  using gps_msgs::msg::GPSStatus;
  using sensor_msgs::msg::NavSatStatus;
  using gpsd_client::gps_h::kStatusDr;
  using gpsd_client::gps_h::kStatusGnssDr;
  using gpsd_client::gps_h::kStatusRtkFix;
  using gpsd_client::gps_h::kStatusRtkFloat;
  using gpsd_client::gps_h::kStatusSim;
  using gpsd_client::gps_h::kStatusTime;
  constexpr uint16_t kGps = GPSStatus::SOURCE_GPS;
  constexpr uint16_t kPoints = GPSStatus::SOURCE_POINTS;
  constexpr uint16_t kNone = GPSStatus::SOURCE_NONE;
  const std::vector<StatusCase> cases = {
    {"GPS", kStatusGps, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX, kGps, kPoints},
    {"DGPS", kStatusDgps, GPSStatus::STATUS_DGPS_FIX, NavSatStatus::STATUS_GBAS_FIX, kGps,
      kPoints},
    {"RTK fixed", kStatusRtkFix, GPSStatus::STATUS_RTK_FIX, NavSatStatus::STATUS_GBAS_FIX,
      kGps, kPoints},
    {"RTK float", kStatusRtkFloat, GPSStatus::STATUS_RTK_FLOAT,
      NavSatStatus::STATUS_GBAS_FIX, kGps, kPoints},
    // Dead reckoning alone has no GNSS in it, so it claims no source.
    {"DR", kStatusDr, GPSStatus::STATUS_DR_FIX, NavSatStatus::STATUS_FIX, kNone, kNone},
    // GNSS aided by dead reckoning is still a GNSS fix.
    {"GNSS+DR", kStatusGnssDr, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX, kGps,
      kPoints},
    {"time only", kStatusTime, GPSStatus::STATUS_TIME_FIX, NavSatStatus::STATUS_FIX, kGps,
      kPoints},
    {"simulated", kStatusSim, GPSStatus::STATUS_SIM_FIX, NavSatStatus::STATUS_FIX, kGps,
      kPoints},
    // Anything else, such as unknown (0) or PPS (9), is a plain fix.
    {"unknown", 0, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX, kGps, kPoints},
    {"PPS", 9, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX, kGps, kPoints},
  };
  for (const StatusCase & c : cases) {
    checkStatus(makeContext(), c);
  }
}

TEST(GpsdParser, LegacyFixSemanticsReportsEveryFixAsAPlainGpsFix)
{
  using gps_msgs::msg::GPSStatus;
  using sensor_msgs::msg::NavSatStatus;
  using gpsd_client::gps_h::kStatusDr;
  using gpsd_client::gps_h::kStatusSim;
  using gpsd_client::gps_h::kStatusTime;
  auto context = makeContext();
  context.legacy_fix_semantics = true;
  const std::vector<StatusCase> cases = {
    {"DR", kStatusDr, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX,
      GPSStatus::SOURCE_GPS, GPSStatus::SOURCE_POINTS},
    {"time only", kStatusTime, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX,
      GPSStatus::SOURCE_GPS, GPSStatus::SOURCE_POINTS},
    {"simulated", kStatusSim, GPSStatus::STATUS_FIX, NavSatStatus::STATUS_FIX,
      GPSStatus::SOURCE_GPS, GPSStatus::SOURCE_POINTS},
  };
  for (const StatusCase & c : cases) {
    checkStatus(context, c);
  }
}

TEST(GpsdParser, OverrideAugmentationSourceReportsSbasWithoutSbasSatellites)
{
  auto context = makeContext();
  context.override_augmentation_source = true;
  auto parser = makeParser(context);

  gps_data_t data = makeThreeDFix();
  setFixStatus(data, kStatusDgps);

  // No SBAS satellite in the skyview, but the override forces SBAS anyway,
  // consistently across both messages.
  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX);

  // The override only affects DGPS reports.
  setFixStatus(data, kStatusGps);
  navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);

  fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, gps_msgs::msg::GPSStatus::STATUS_FIX);
}

TEST(GpsdParser, NavSatFixStampUsesGpsTimeWhenEnabled)
{
  gps_data_t data = makeThreeDFix();

  auto context = makeContext();
  context.use_gps_time = true;
  auto fix = makeParser(context)->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->header.stamp.sec, 1700000000);
  EXPECT_EQ(fix->header.stamp.nanosec, 500000000u);

  fix = makeParser()->parseNavSatFix(data, rclcpp::Time(42, 7));
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->header.stamp.sec, 42);
  EXPECT_EQ(fix->header.stamp.nanosec, 7u);
}

TEST(GpsdParser, NavSatFixStampFallsBackWithoutGpsTime)
{
  // A receiver without a fix often has no time either, and then GPSd leaves
  // the fix time unset. use_gps_time cannot stamp with it.
  gps_data_t data = makeThreeDFix();
  data.fix.mode = MODE_NO_FIX;
  data.fix.time.tv_sec = 0;
  data.fix.time.tv_nsec = 0;

  auto context = makeContext();
  context.use_gps_time = true;
  auto fix = makeParser(context)->parseNavSatFix(data, rclcpp::Time(42, 7));
  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->header.stamp.sec, 42);
  EXPECT_EQ(fix->header.stamp.nanosec, 7u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
