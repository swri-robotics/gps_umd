/// Tests for the raw parser and the generated fill code behind it.
///
/// Normal include order here (gpsd_client headers first, which pull gps.h in
/// behind the message headers). The gps.h-first case has its own translation
/// unit in test_gpsd_raw_include_order.cpp, because it cannot include anything
/// that reaches the legacy GPSFix/GPSStatus messages -- those still collide
/// with gps.h and always have.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <memory>

#include <gpsd_client/gpsd_parser_factory.hpp>
#include <gpsd_client/gpsd_raw_parser.hpp>

#include "gpsd_json_fixture.hpp"

namespace
{

// gpsd renamed STATUS_FIX to STATUS_GPS in 3.23; the value (1) never changed.
#ifdef STATUS_GPS
constexpr int kStatusGps = STATUS_GPS;
#else
constexpr int kStatusGps = STATUS_FIX;
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

std::unique_ptr<gpsd_client::GpsdRawParser> makeParser()
{
  return gpsd_client::GpsdParserFactory::createRaw(makeContext());
}

}  // namespace

TEST(GpsdRawParser, SelectedMessageMatchesTheBuildsApiVersion)
{
  // The whole point of the versioned messages: the type compiled in must be
  // the one named after this libgps.
  EXPECT_EQ(GPSD_RAW_FILL_MAJOR, GPSD_API_MAJOR_VERSION);
  EXPECT_EQ(GPSD_RAW_FILL_MINOR, GPSD_API_MINOR_VERSION);
  EXPECT_EQ(gpsd_client::GpsdRawMsg::SET_LATLON, LATLON_SET);
  EXPECT_EQ(gpsd_client::GpsdRawMsg::SET_UNION & static_cast<uint64_t>(UNION_SET),
            static_cast<uint64_t>(UNION_SET));
  // Not EXPECT_EQ: the message is generated from the last rev of its API
  // pair, which may know about more mask bits than this build's gps.h (see
  // test_gpsd_raw_include_order.cpp for the full explanation).
  EXPECT_GE(gpsd_client::GpsdRawMsg::SET_HIGHEST_BIT,
            static_cast<uint64_t>(SET_HIGH_BIT));
}

TEST(GpsdRawParser, StampsAndFramesTheHeader)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(42, 7));

  EXPECT_EQ(msg.header.frame_id, "gps");
  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_EQ(msg.header.stamp.nanosec, 7u);
}

TEST(GpsdRawParser, CopiesScalarsAndNestedStructs)
{
  gps_data_t data = gpsd_client::test::makeThreeDFixFromJson();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(42, 0));

  EXPECT_DOUBLE_EQ(msg.fix.latitude, 29.44);
  EXPECT_DOUBLE_EQ(msg.fix.longitude, -98.61);
  EXPECT_DOUBLE_EQ(msg.fix.altitude, 250.0);
  EXPECT_DOUBLE_EQ(msg.fix.speed, 2.5);
  EXPECT_EQ(msg.fix.mode, MODE_3D);

  EXPECT_DOUBLE_EQ(msg.dop.hdop, 1.2);
  EXPECT_DOUBLE_EQ(msg.dop.pdop, 1.1);
  EXPECT_DOUBLE_EQ(msg.dop.gdop, 1.5);

  // timespec_t -> builtin_interfaces/Time keeps the nanoseconds.
  EXPECT_EQ(msg.fix.time.sec, 1700000000);
  EXPECT_EQ(msg.fix.time.nanosec, 500000000u);
  EXPECT_EQ(msg.online.sec, 100);
}

TEST(GpsdRawParser, SkyviewIsTruncatedToTheValidCount)
{
  // The array the generator deliberately leaves alone: its length lives in a
  // sibling field, so publishing the whole fixed array would emit MAXCHANNELS
  // entries of uninitialised satellites.
  gps_data_t data = gpsd_client::test::makeThreeDFixFromJson();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(42, 0));

  ASSERT_EQ(msg.skyview.size(), 3u);
  EXPECT_LT(msg.skyview.size(), static_cast<std::size_t>(MAXCHANNELS));

  EXPECT_EQ(msg.skyview[0].prn, 10);
  EXPECT_DOUBLE_EQ(msg.skyview[0].elevation, 30.0);
  EXPECT_DOUBLE_EQ(msg.skyview[0].azimuth, 100.0);
  EXPECT_DOUBLE_EQ(msg.skyview[0].ss, 40.0);
  EXPECT_TRUE(msg.skyview[0].used);
  EXPECT_EQ(msg.skyview[2].prn, 12);
  EXPECT_FALSE(msg.skyview[2].used);
}

TEST(GpsdRawParser, SkyviewCountIsClampedAgainstGarbage)
{
  // satellites_visible is a plain int; a truncated or stale report can leave
  // it negative or past the end of the array, and neither may be trusted as a
  // loop bound.
  gps_data_t data = gpsd_client::test::makeEmptyData();

  data.satellites_visible = -1;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).skyview.size(), 0u);

  data.satellites_visible = MAXCHANNELS + 500;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).skyview.size(),
            static_cast<std::size_t>(MAXCHANNELS));
}

TEST(GpsdRawParser, PreservesNanRatherThanZeroing)
{
  // gpsd uses NaN for "unknown" throughout. A raw message that reported 0.0
  // instead would be asserting a measurement that was never made.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_TRUE(std::isnan(msg.fix.latitude));
  EXPECT_TRUE(std::isnan(msg.fix.longitude));
  EXPECT_TRUE(std::isnan(msg.dop.hdop));
}

TEST(GpsdRawParser, CarriesTheSetMaskVerbatim)
{
  // D5/D10: the mask is copied undecoded, so a consumer can tell that gpsd
  // reported something this message does not carry -- AIS above all.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = LATLON_SET | AIS_SET;

  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.set, static_cast<uint64_t>(LATLON_SET | AIS_SET));
  EXPECT_TRUE(msg.set & gpsd_client::GpsdRawMsg::SET_AIS);
}

TEST(GpsdRawParser, FixStatusIsCarriedWhereverThisVersionKeepsIt)
{
  // API 9 keeps the fix status in gps_data_t, API 10+ in gps_fix_t. The raw
  // message mirrors its own version's layout rather than normalising, so the
  // field simply lives in a different sub-message either way.
  gps_data_t data = gpsd_client::test::makeEmptyData();
#if GPSD_API_MAJOR_VERSION >= 10
  data.fix.status = kStatusGps;
#else
  data.status = kStatusGps;
#endif

  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
#if GPSD_API_MAJOR_VERSION >= 10
  EXPECT_EQ(msg.fix.status, kStatusGps);
#else
  EXPECT_EQ(msg.status, kStatusGps);
#endif
}

// --- Tier B arrays -------------------------------------------------------

// gps_data_t has carried `devices` since API 9, so this needs no guard --
// unlike the imu[] and fixsource_t tests below, which arrived later.
TEST(GpsdRawParser, DeviceListIsTrimmedToNdevices)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  const std::size_t capacity =
      sizeof(data.devices.list) / sizeof(data.devices.list[0]);

  data.devices.ndevices = 2;
  snprintf(data.devices.list[0].path, sizeof(data.devices.list[0].path),
           "/dev/ttyS0");
  snprintf(data.devices.list[1].path, sizeof(data.devices.list[1].path),
           "/dev/ttyS1");

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.devices.list.size(), 2u);
  EXPECT_LT(msg.devices.list.size(), capacity);
  EXPECT_EQ(msg.devices.list[0].path, "/dev/ttyS0");
  EXPECT_EQ(msg.devices.list[1].path, "/dev/ttyS1");
  EXPECT_EQ(msg.devices.ndevices, 2);

  // Same garbage-clamping as skyview: ndevices is a plain int.
  data.devices.ndevices = -1;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).devices.list.size(), 0u);
  data.devices.ndevices = static_cast<int>(capacity) + 100;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).devices.list.size(),
            capacity);
}

#if GPSD_API_MAJOR_VERSION >= 12
TEST(GpsdRawParser, ImuIsTerminatedByAnEmptyMsg)
{
  /* imu[] carries no count. gpsd's own JSON dumper walks it until
   * attitude_t::msg is empty, and the u-blox driver stamps msg on every entry
   * it fills, so that terminator is the only authority on how many are real.
   */
  gps_data_t data = gpsd_client::test::makeEmptyData();
  const std::size_t max_imu = sizeof(data.imu) / sizeof(data.imu[0]);

  // Nothing stamped: nothing published, rather than ten empty entries.
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).imu.size(), 0u);

  snprintf(data.imu[0].msg, sizeof(data.imu[0].msg), "UBX-ESF-RAW");
  snprintf(data.imu[1].msg, sizeof(data.imu[1].msg), "UBX-ESF-RAW");
  data.imu[1].gyro_x = 1.5;
  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.imu.size(), 2u);
  EXPECT_EQ(msg.imu[0].msg, "UBX-ESF-RAW");
  EXPECT_DOUBLE_EQ(msg.imu[1].gyro_x, 1.5);

  // A gap terminates: entry 3 is stamped but unreachable past the empty 2.
  snprintf(data.imu[3].msg, sizeof(data.imu[3].msg), "UBX-ESF-RAW");
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).imu.size(), 2u);

  // All ten stamped: bounded by the array, never past it.
  for (std::size_t i = 0; i < max_imu; ++i)
  {
    snprintf(data.imu[i].msg, sizeof(data.imu[i].msg), "UBX-ESF-RAW");
  }
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).imu.size(), max_imu);
}
#endif

#if GPSD_API_MAJOR_VERSION >= 14
TEST(GpsdRawParser, PointerMembersAreNotPublished)
{
  // fixsource_t's server/port/device are const char* into caller memory --
  // gps_open() stores the host argument verbatim, and gpsd_client passes a
  // c_str() that dangles once start() returns. spec carries the same
  // information as a real array and is what gets published.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  snprintf(data.source.spec, sizeof(data.source.spec), "localhost:2947");

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.source.spec, "localhost:2947");
}
#endif  // fixsource_t reached gps_data_t in API 14

// --- Tier C union dispatch (D16) -----------------------------------------

TEST(GpsdRawParser, ReportUnionFillsOnlyTheArmTheMaskNames)
{
  // gps_data_t packs its report arms into a union; the set mask says which is
  // live. Filling any other would be reading an inactive union member, so the
  // arms are 0-or-1 arrays and only the named one is ever non-empty.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.rtcm3.type = 1005;
  data.set = RTCM3_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.rtcm3.size(), 1u) << "RTCM3_SET was set";
  EXPECT_EQ(msg.rtcm3[0].type, 1005u);

  // Every other arm stays empty -- that is what says "not this kind of report".
  EXPECT_TRUE(msg.rtcm2.empty());
  EXPECT_TRUE(msg.subframe.empty());
  EXPECT_TRUE(msg.version.empty());
  EXPECT_TRUE(msg.error.empty());
}

TEST(GpsdRawParser, NoReportBitMeansNoArm)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.rtcm3.type = 1005;
  data.set = 0;                 // nothing reported

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_TRUE(msg.rtcm3.empty())
      << "an arm must not be filled just because the struct holds stale bytes";
}

TEST(GpsdRawParser, ErrorArmIsAStringFromTheMask)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  snprintf(data.error, sizeof(data.error), "no such device");
  data.set = ERROR_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.error.size(), 1u);
  EXPECT_EQ(msg.error[0], "no such device");
}

TEST(GpsdRawParser, Rtcm3TypeSelectsItsArm)
{
  // The inner union: rtcm3_t::type names the arm, and the arm names encode
  // the type (rtcm3_1005 <-> 1005), so the mapping is generated, not curated.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = RTCM3_SET;
  data.rtcm3.type = 1005;
  data.rtcm3.rtcmtypes.rtcm3_1005.station_id = 42;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.rtcm3.size(), 1u);
  ASSERT_EQ(msg.rtcm3[0].rtcmtypes.rtcm3_1005.size(), 1u);
  EXPECT_EQ(msg.rtcm3[0].rtcmtypes.rtcm3_1005[0].station_id, 42u);
  // rtcm3_1001 exists in every supported version; 1230 only from API 12, so
  // pick a sibling that is always present for the "others stay empty" check.
  EXPECT_TRUE(msg.rtcm3[0].rtcmtypes.rtcm3_1001.empty());
  EXPECT_TRUE(msg.rtcm3[0].rtcmtypes.rtcm3_1003.empty());
}

#if GPSD_API_MAJOR_VERSION >= 13
TEST(GpsdRawParser, Rtcm3MsmTypesShareOneArm)
{
  // ~43 Multiple Signal Message types fold into rtcm3_msm, exactly as gpsd's
  // own dumper folds them into one handler.
  for (unsigned type : {1071u, 1077u, 1097u, 1127u})
  {
    gps_data_t data = gpsd_client::test::makeEmptyData();
    data.set = RTCM3_SET;
    data.rtcm3.type = type;
    data.rtcm3.rtcmtypes.rtcm3_msm.station_id = 7;

    auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
    ASSERT_EQ(msg.rtcm3.size(), 1u);
    ASSERT_EQ(msg.rtcm3[0].rtcmtypes.rtcm3_msm.size(), 1u) << "type " << type;
    EXPECT_EQ(msg.rtcm3[0].rtcmtypes.rtcm3_msm[0].station_id, 7u);
  }
}
#endif  // rtcm3_msm was added in API 13

TEST(GpsdRawParser, UnknownRtcm3TypeFallsBackToRawBytes)
{
  // gpsd keeps whatever it could not decode in `data`, so that is the default
  // arm rather than a type of its own.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = RTCM3_SET;
  data.rtcm3.type = 9999;       // not a type gpsd decodes

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.rtcm3.size(), 1u);
  EXPECT_TRUE(msg.rtcm3[0].rtcmtypes.rtcm3_1005.empty());
  EXPECT_EQ(msg.rtcm3[0].rtcmtypes.data.size(),
            sizeof(data.rtcm3.rtcmtypes.data));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
