#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <gpsd_client/gpsd_parser_factory.hpp>

namespace
{

// The tests, like the parsers, can only compile against the single installed
// libgps header, so they exercise whichever parser the factory selects for
// GPSD_API_MAJOR_VERSION. The two version-dependent details below (where the
// fix status lives and what the DGPS status macro is called) are isolated
// here.
void setFixStatus(gps_data_t& data, int status)
{
#if GPSD_API_MAJOR_VERSION >= 10
  data.fix.status = status;
#else
  data.status = status;
#endif
}

#ifdef STATUS_DGPS_FIX
constexpr int kStatusDgps = STATUS_DGPS_FIX;
#else
constexpr int kStatusDgps = STATUS_DGPS;
#endif

// A plain GPS fix: STATUS_FIX until gpsd renamed it to STATUS_GPS.
#ifdef STATUS_GPS
constexpr int kStatusGps = STATUS_GPS;
#else
constexpr int kStatusGps = STATUS_FIX;
#endif

/* gps.h defines STATUS_* macros whose names collide with the ROS message
 * constants (e.g. STATUS_FIX in gpsd < 3.23), so the expectations below use
 * the messages' integer values with the symbolic name in a comment -- the
 * same convention the parser sources use.
 */

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
  data.fix.altitude = 250.0;
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
  for (int i = 0; i < 3; ++i)
  {
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
    const gpsd_client::ParserContext& context = makeContext())
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

  EXPECT_EQ(fix.status.status, 0 /* GPSStatus::STATUS_FIX */);
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
  EXPECT_EQ(fix->status.status, 0 /* NavSatStatus::STATUS_FIX */);
  EXPECT_EQ(fix->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
  EXPECT_DOUBLE_EQ(fix->latitude, 29.44);
  EXPECT_DOUBLE_EQ(fix->longitude, -98.61);
  EXPECT_DOUBLE_EQ(fix->altitude, 250.0);
  EXPECT_DOUBLE_EQ(fix->position_covariance[0], 1.5);
  EXPECT_DOUBLE_EQ(fix->position_covariance[4], 2.5);
  EXPECT_DOUBLE_EQ(fix->position_covariance[8], 3.5);
  EXPECT_EQ(fix->position_covariance_type,
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
}

TEST(GpsdParser, TwoDFixHasNanAltitude)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  data.fix.mode = MODE_2D;

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));

  EXPECT_EQ(fix.status.status, 0 /* GPSStatus::STATUS_FIX */);
  EXPECT_DOUBLE_EQ(fix.latitude, 29.44);
  EXPECT_TRUE(std::isnan(fix.altitude));
}

TEST(GpsdParser, NoFixSetsNoFixStatus)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  data.fix.mode = MODE_NO_FIX;

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, -1 /* GPSStatus::STATUS_NO_FIX */);
  // Position fields are only filled in when there is a fix.
  EXPECT_DOUBLE_EQ(fix.latitude, 0.0);

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());  // variance check is off
  EXPECT_EQ(navsat_fix->status.status, -1 /* NavSatStatus::STATUS_NO_FIX */);
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
  EXPECT_EQ(fix.status.status, -1 /* GPSStatus::STATUS_NO_FIX */);
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
  EXPECT_EQ(navsat_fix->status.service,
            sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
            sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS);
  EXPECT_EQ(navsat_fix->status.status, 1 /* NavSatStatus::STATUS_SBAS_FIX */);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, 1 /* GPSStatus::STATUS_SBAS_FIX */);
}

TEST(GpsdParser, DgpsStatusWithoutSbas)
{
  auto parser = makeParser();
  gps_data_t data = makeThreeDFix();
  setFixStatus(data, kStatusDgps);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, 18 /* GPSStatus::STATUS_DGPS_FIX */);

  auto navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, 2 /* NavSatStatus::STATUS_GBAS_FIX */);
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
  EXPECT_EQ(navsat_fix->status.status, 1 /* NavSatStatus::STATUS_SBAS_FIX */);

  gps_msgs::msg::GPSFix fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, 1 /* GPSStatus::STATUS_SBAS_FIX */);

  // The override only affects DGPS reports.
  setFixStatus(data, kStatusGps);
  navsat_fix = parser->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.status, 0 /* NavSatStatus::STATUS_FIX */);

  fix = parser->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, 0 /* GPSStatus::STATUS_FIX */);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
