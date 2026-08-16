/// Tests for the tier-1 JSON fixture harness, and for the parsers driven by it.
///
/// Two jobs:
///   1. Prove the harness itself behaves -- that gps_unpack() populates what we
///      claim, and that the version-portability guarantees in the header hold
///      against whichever libgps this build linked.
///   2. Exercise the existing parsers with data decoded by libgps rather than
///      assigned by hand, which is the closer analogue of a live session.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <gpsd_client/gpsd_parser_factory.hpp>

#include "gpsd_json_fixture.hpp"

namespace
{

using gpsd_client::test::Dop;
using gpsd_client::test::Satellite;
using gpsd_client::test::Tpv;

// gpsd renamed STATUS_FIX to STATUS_GPS in 3.23 and STATUS_DGPS_FIX to
// STATUS_DGPS in 3.25. The numeric values did not change (1 and 2), but the
// spellings must still be resolved by the preprocessor.
#ifdef STATUS_GPS
constexpr int kStatusGps = STATUS_GPS;
#else
constexpr int kStatusGps = STATUS_FIX;
#endif

#ifdef STATUS_DGPS_FIX
constexpr int kStatusDgps = STATUS_DGPS_FIX;
#else
constexpr int kStatusDgps = STATUS_DGPS;
#endif

gpsd_client::ParserContext makeContext()
{
  gpsd_client::ParserContext context;
  context.frame_id = "gps";
  context.use_gps_time = false;
  context.check_fix_by_variance = false;
  context.override_augmentation_source = false;
  return context;
}

std::unique_ptr<gpsd_client::GpsdParser> makeParser(
    const gpsd_client::ParserContext& context = makeContext())
{
  return gpsd_client::GpsdParserFactory::create(context);
}

std::vector<Satellite> threeSatellites()
{
  std::vector<Satellite> satellites;
  for (int i = 0; i < 3; ++i)
  {
    Satellite sat;
    sat.prn = 10 + i;
    sat.elevation = 30 + i;
    sat.azimuth = 100 + i;
    sat.snr = 40 + i;
    sat.gnssid = GNSSID_GPS;
    sat.used = i < 2;
    satellites.push_back(sat);
  }
  return satellites;
}

}  // namespace

// --- The harness itself ----------------------------------------------------

TEST(JsonFixture, EmptyDataUsesNanSentinels)
{
  // gps_open() clears the fix rather than zeroing it, so "unknown" is NAN.
  // A zero-filled struct would make the parser look correct here while
  // disagreeing with a live client.
  gps_data_t data = gpsd_client::test::makeEmptyData();

  EXPECT_TRUE(std::isnan(data.fix.latitude));
  EXPECT_TRUE(std::isnan(data.fix.longitude));
  EXPECT_TRUE(std::isnan(data.fix.altitude));
  EXPECT_TRUE(std::isnan(data.dop.hdop));
  EXPECT_EQ(data.fix.mode, MODE_NOT_SEEN);
  EXPECT_EQ(data.set, 0u);
  EXPECT_EQ(data.satellites_used, 0);
}

TEST(JsonFixture, UnpackCannotReportMalformedJson)
{
  // Pins a libgps limitation rather than a desirable behavior: through gpsd
  // 3.27.5, gps_unpack() breaks out of its segment loop on a parse failure and
  // still returns 0. Fixtures therefore cannot rely on the status to catch a
  // typo -- hence the builders, and hence asserting on decoded values.
  //
  // gpsd master has added error propagation. When a release ships it, this
  // test starts failing, which is the intended signal to tighten unpack().
  gps_data_t data = gpsd_client::test::makeEmptyData();

  EXPECT_EQ(gpsd_client::test::unpack(data, "{\"class\":\"TPV\",\"mode\":}"), 0);
  EXPECT_EQ(gpsd_client::test::unpack(data, "{\"class\":\"TPV\",\"mode\":3"), 0);
  EXPECT_EQ(gpsd_client::test::unpack(data, "{\"class\":\"TPV\",\"lat\":\"nan\"}"), 0);

  // What *is* observable: nothing was decoded from any of them.
  EXPECT_EQ(data.fix.mode, MODE_NOT_SEEN);
  EXPECT_TRUE(std::isnan(data.fix.latitude));
}

TEST(JsonFixture, TpvPopulatesFix)
{
  Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.time = "2023-11-14T22:13:20.500Z";
  tpv.latitude = 29.44;
  tpv.longitude = -98.61;
  tpv.altitude = 250.0;
  tpv.speed = 2.5;

  gps_data_t data = gpsd_client::test::makeEmptyData();
  ASSERT_EQ(gpsd_client::test::unpack(data, gpsd_client::test::tpvJson(tpv)), 0);

  EXPECT_EQ(data.fix.mode, MODE_3D);
  EXPECT_DOUBLE_EQ(data.fix.latitude, 29.44);
  EXPECT_DOUBLE_EQ(data.fix.longitude, -98.61);
  EXPECT_DOUBLE_EQ(data.fix.speed, 2.5);

  // The ISO 8601 string went through libgps' own time conversion.
  EXPECT_EQ(data.fix.time.tv_sec, 1700000000);
  EXPECT_EQ(data.fix.time.tv_nsec, 500000000);

  // Keys left at NAN were omitted entirely, so they stay unknown.
  EXPECT_TRUE(std::isnan(data.fix.track));
}

TEST(JsonFixture, SkyPopulatesSkyview)
{
  // Regression guard for the nSat quirk documented in skyJson(): from gpsd
  // 3.24 on, a SKY report without "nSat" is decoded as "no satellites" and
  // every entry here would silently vanish.
  Dop dop;
  dop.hdop = 1.2;
  dop.pdop = 1.1;

  gps_data_t data = gpsd_client::test::makeEmptyData();
  ASSERT_EQ(gpsd_client::test::unpack(
                data, gpsd_client::test::skyJson(threeSatellites(), dop)),
            0);

  ASSERT_EQ(data.satellites_visible, 3);
  EXPECT_EQ(data.satellites_used, 2);

  EXPECT_EQ(data.skyview[0].PRN, 10);
  EXPECT_DOUBLE_EQ(data.skyview[0].elevation, 30.0);
  EXPECT_DOUBLE_EQ(data.skyview[0].azimuth, 100.0);
  EXPECT_DOUBLE_EQ(data.skyview[0].ss, 40.0);
  EXPECT_TRUE(data.skyview[0].used);
  EXPECT_FALSE(data.skyview[2].used);

  EXPECT_DOUBLE_EQ(data.dop.hdop, 1.2);
  EXPECT_DOUBLE_EQ(data.dop.pdop, 1.1);
}

TEST(JsonFixture, ConcatenatedReportsAccumulate)
{
  // gpsd packs several reports into one write; gps_unpack loops over the
  // segments, and state accumulates across them exactly as in a live session.
  Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.latitude = 1.0;

  gps_data_t data = gpsd_client::test::makeEmptyData();
  ASSERT_EQ(gpsd_client::test::unpack(
                data,
                gpsd_client::test::tpvJson(tpv) +
                    gpsd_client::test::skyJson(threeSatellites())),
            0);

  EXPECT_DOUBLE_EQ(data.fix.latitude, 1.0);  // from the TPV
  EXPECT_EQ(data.satellites_visible, 3);     // from the SKY
}

TEST(JsonFixture, StatusLandsWhereThisVersionKeepsIt)
{
  // The payoff for going through libgps: gpsd 3.20 parses "status" into
  // gps_data_t.status and 3.21+ into gps_data_t.fix.status. The fixture says
  // nothing about which, and the parser reads the right one either way.
  Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.status = kStatusDgps;
  tpv.latitude = 29.44;
  tpv.longitude = -98.61;

  gps_data_t data = gpsd_client::test::makeEmptyData();
  ASSERT_EQ(gpsd_client::test::unpack(data, gpsd_client::test::tpvJson(tpv)), 0);
  data.online.tv_sec = 100;

  gps_msgs::msg::GPSFix fix =
      makeParser()->parseGpsFix(data, rclcpp::Time(42, 0));
  EXPECT_EQ(fix.status.status, 18 /* GPSStatus::STATUS_DGPS_FIX */);
}

// --- Parsers, driven by libgps-decoded data --------------------------------

TEST(JsonFixtureParser, ThreeDFixPopulatesGpsFix)
{
  gps_data_t data = gpsd_client::test::makeThreeDFixFromJson();
  gps_msgs::msg::GPSFix fix =
      makeParser()->parseGpsFix(data, rclcpp::Time(42, 0));

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

TEST(JsonFixtureParser, ThreeDFixPopulatesNavSatFix)
{
  gps_data_t data = gpsd_client::test::makeThreeDFixFromJson();
  auto fix = makeParser()->parseNavSatFix(data, rclcpp::Time(42, 0));

  ASSERT_TRUE(fix.has_value());
  EXPECT_EQ(fix->header.frame_id, "gps");
  EXPECT_EQ(fix->status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
  EXPECT_DOUBLE_EQ(fix->latitude, 29.44);
  EXPECT_DOUBLE_EQ(fix->longitude, -98.61);
  EXPECT_DOUBLE_EQ(fix->altitude, 250.0);
  EXPECT_DOUBLE_EQ(fix->position_covariance[0], 1.5);
  EXPECT_DOUBLE_EQ(fix->position_covariance[4], 2.5);
  EXPECT_DOUBLE_EQ(fix->position_covariance[8], 3.5);
}

TEST(JsonFixtureParser, SbasSatelliteYieldsSbasStatus)
{
  std::vector<Satellite> satellites = threeSatellites();
  satellites[1].gnssid = GNSSID_GLO;
  satellites[2].gnssid = GNSSID_SBAS;
  satellites[2].used = true;

  Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.status = kStatusDgps;
  tpv.latitude = 29.44;
  tpv.longitude = -98.61;
  tpv.epx = 1.5;
  tpv.epy = 2.5;
  tpv.epv = 3.5;

  gps_data_t data = gpsd_client::test::makeEmptyData();
  ASSERT_EQ(gpsd_client::test::unpack(data,
                                      gpsd_client::test::tpvJson(tpv) +
                                          gpsd_client::test::skyJson(satellites)),
            0);
  data.online.tv_sec = 100;

  auto navsat_fix = makeParser()->parseNavSatFix(data, rclcpp::Time(42, 0));
  ASSERT_TRUE(navsat_fix.has_value());
  EXPECT_EQ(navsat_fix->status.service,
            sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS);
  EXPECT_EQ(navsat_fix->status.status, 1 /* NavSatStatus::STATUS_SBAS_FIX */);
}

TEST(JsonFixtureParser, NanVarianceFromOmittedKeysIsRejected)
{
  // A TPV that never mentions epx/epy/epv leaves them NAN, which is exactly
  // the stale-fix case check_fix_by_variance exists to catch. Only reachable
  // because makeEmptyData() clears rather than zeroes.
  Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.status = kStatusGps;
  tpv.latitude = 29.44;
  tpv.longitude = -98.61;

  gps_data_t data = gpsd_client::test::makeEmptyData();
  ASSERT_EQ(gpsd_client::test::unpack(data, gpsd_client::test::tpvJson(tpv)), 0);
  data.online.tv_sec = 100;

  auto context = makeContext();
  context.check_fix_by_variance = true;
  EXPECT_FALSE(
      makeParser(context)->parseNavSatFix(data, rclcpp::Time(42, 0)).has_value());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
