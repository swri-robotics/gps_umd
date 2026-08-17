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
#include <cstring>
#include <memory>
#include <string>

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

TEST(GpsdRawParser, AnUnterminatedCharArrayStopsAtTheEndOfTheArray)
{
  // gpsd's char[N] members carry no guarantee of a NUL: a driver that fills
  // the array exactly leaves no room for one, and gpsd's own code reads these
  // with bounded calls for that reason. Constructing the ROS string from a
  // plain strlen would run past the array into whatever the struct puts next
  // -- reading uninitialised bytes at best, off the end of the object at
  // worst -- so the fill code bounds it with strnlen(..., sizeof).
  gps_data_t data = gpsd_client::test::makeEmptyData();
  std::memset(data.dev.path, 'x', sizeof(data.dev.path));

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_EQ(sizeof(data.dev.path), msg.dev.path.size());
  EXPECT_EQ(std::string(sizeof(data.dev.path), 'x'), msg.dev.path);
}

TEST(GpsdRawParser, ATerminatedCharArrayStopsAtTheTerminator)
{
  // The other half of the pair: bounding by sizeof must not also mean
  // publishing the padding after a short, properly terminated string.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  std::memset(data.dev.path, '\0', sizeof(data.dev.path));
  snprintf(data.dev.path, sizeof(data.dev.path), "/dev/ttyS0");

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_EQ("/dev/ttyS0", msg.dev.path);
}

// --- Tier C union dispatch (D16) -----------------------------------------

TEST(GpsdRawParser, ReportUnionFillsOnlyTheArmTheMaskNames)
{
  // gps_data_t packs its report arms into a union; the set mask says which is
  // live. Filling any other would be reading an inactive union member, so the
  // arms are 0-or-1 arrays and only the named one is ever non-empty.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  snprintf(data.dev.path, sizeof(data.dev.path), "/dev/ttyS0");
  data.set = DEVICE_SET | VERSION_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.version.size(), 1u) << "VERSION_SET was set";

  // Every other arm stays empty -- that is what says "not this kind of report".
  EXPECT_TRUE(msg.subframe.empty());
  EXPECT_TRUE(msg.error.empty());
  EXPECT_TRUE(msg.osc.empty());
}

TEST(GpsdRawParser, RtcmIsNotCarriedInTheRawMessage)
{
  /* RTCM has its own topic. The raw message still reports the mask verbatim,
   * so a subscriber can see that an RTCM report arrived and go look at
   * gpsd_rtcm2 / gpsd_rtcm3 for it -- the same contract as AIS.
   */
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.rtcm3.type = 1005;
  data.set = RTCM3_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_TRUE(msg.set & gpsd_client::GpsdRawMsg::SET_RTCM3)
      << "the mask must still say an RTCM3 report was seen";
}

TEST(GpsdRawParser, NoReportBitMeansNoRtcmMessage)
{
  // Nothing is published just because the union holds stale bytes: the mask
  // is the only thing that makes reading that arm defined.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.rtcm3.type = 1005;
  data.set = 0;                 // nothing reported

  EXPECT_FALSE(makeParser()->parseRtcm3(data, rclcpp::Time(0, 0)).has_value());
  EXPECT_FALSE(makeParser()->parseRtcm2(data, rclcpp::Time(0, 0)).has_value());
}

TEST(GpsdRawParser, RtcmMessagesCarryAHeader)
{
  // They are published in their own right now, so they need a stamp and frame.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = RTCM3_SET;
  data.rtcm3.type = 1005;

  auto msg = makeParser()->parseRtcm3(data, rclcpp::Time(42, 7));
  ASSERT_TRUE(msg.has_value());
  EXPECT_EQ(msg->header.frame_id, "gps");
  EXPECT_EQ(msg->header.stamp.sec, 42);
  EXPECT_EQ(msg->type, 1005u);
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

  auto msg = makeParser()->parseRtcm3(data, rclcpp::Time(0, 0));
  ASSERT_TRUE(msg.has_value());
  ASSERT_EQ(msg->rtcmtypes.rtcm3_1005.size(), 1u);
  EXPECT_EQ(msg->rtcmtypes.rtcm3_1005[0].station_id, 42u);
  // rtcm3_1001 exists in every supported version; 1230 only from API 12, so
  // pick a sibling that is always present for the "others stay empty" check.
  EXPECT_TRUE(msg->rtcmtypes.rtcm3_1001.empty());
  EXPECT_TRUE(msg->rtcmtypes.rtcm3_1003.empty());
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

    auto msg = makeParser()->parseRtcm3(data, rclcpp::Time(0, 0));
    ASSERT_TRUE(msg.has_value()) << "type " << type;
    ASSERT_EQ(msg->rtcmtypes.rtcm3_msm.size(), 1u) << "type " << type;
    EXPECT_EQ(msg->rtcmtypes.rtcm3_msm[0].station_id, 7u);
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

  auto msg = makeParser()->parseRtcm3(data, rclcpp::Time(0, 0));
  ASSERT_TRUE(msg.has_value());
  EXPECT_TRUE(msg->rtcmtypes.rtcm3_1005.empty());
  EXPECT_EQ(msg->rtcmtypes.data.size(), sizeof(data.rtcm3.rtcmtypes.data));
}

TEST(GpsdRawParser, Rtcm2TypeSelectsItsArm)
{
  // rtcm2_t's union is anonymous, so its arms sit alongside the discriminator
  // rather than one level down. The names give no hint of the type, so the
  // mapping is a curated table read off gpsd's own dumper (D16).
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = RTCM2_SET;
  data.rtcm2.type = 14;                 // GPS time of week
  data.rtcm2.gpstime.week = 2100;

  auto msg = makeParser()->parseRtcm2(data, rclcpp::Time(0, 0));
  ASSERT_TRUE(msg.has_value());
  ASSERT_EQ(msg->gpstime.size(), 1u);
  EXPECT_EQ(msg->gpstime[0].week, 2100u);
  EXPECT_TRUE(msg->gps_ranges.empty());
  EXPECT_TRUE(msg->almanac.empty());
}

TEST(GpsdRawParser, Rtcm2SharedArmsMapToTheSameField)
{
  // Types 1 and 9 carry the same payload, so both select gps_ranges.
  for (unsigned type : {1u, 9u})
  {
    gps_data_t data = gpsd_client::test::makeEmptyData();
    data.set = RTCM2_SET;
    data.rtcm2.type = type;
    data.rtcm2.gps_ranges.nentries = 3;

    auto msg = makeParser()->parseRtcm2(data, rclcpp::Time(0, 0));
    ASSERT_TRUE(msg.has_value());
    ASSERT_EQ(msg->gps_ranges.size(), 1u) << "type " << type;
    EXPECT_EQ(msg->gps_ranges[0].nentries, 3u);
  }
}

TEST(GpsdRawParser, Rtcm2UndecodedTypeKeepsRawWords)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = RTCM2_SET;
  data.rtcm2.type = 99;                 // not a type gpsd decodes

  auto msg = makeParser()->parseRtcm2(data, rclcpp::Time(0, 0));
  ASSERT_TRUE(msg.has_value());
  EXPECT_TRUE(msg->gpstime.empty());
  EXPECT_EQ(msg->words.size(),
            sizeof(data.rtcm2.words) / sizeof(data.rtcm2.words[0]));
}

#if GPSD_API_MAJOR_VERSION >= 10
TEST(GpsdRawParser, Rtcm2DeadArmsAreNeverFilled)
{
  /* rtcm2_18 .. rtcm2_24 are declared in gps.h and written by nothing in
   * gpsd -- no driver, no daemon code. For types 18-22 gpsd fills `rtk` and
   * `ref_sta`, which are not union members at all and so are published as
   * ordinary fields. Filling a dead arm would copy uninitialised union bytes
   * and assert a decode that never happened.
   */
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = RTCM2_SET;
  data.rtcm2.type = 18;
  data.rtcm2.rtk.nentries = 5;          // what gpsd would really fill

  auto msg = makeParser()->parseRtcm2(data, rclcpp::Time(0, 0));
  ASSERT_TRUE(msg.has_value());
  EXPECT_TRUE(msg->rtcm2_18.empty());
  EXPECT_TRUE(msg->rtcm2_19.empty());
  // ... while the field gpsd actually populates comes through as usual.
  EXPECT_EQ(msg->rtk.nentries, 5u);
}
#endif  // rtcm2_t gained rtk/ref_sta and the rtcm2_NN arms at API 10

TEST(GpsdRawParser, SubframeNumberSelectsItsArm)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = SUBFRAME_SET;
  data.subframe.subframe_num = 1;       // clock corrections / health
  data.subframe.sub1.WN = 2100;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.subframe.size(), 1u);
  ASSERT_EQ(msg.subframe[0].sub1.size(), 1u);
  EXPECT_EQ(msg.subframe[0].sub1[0].wn, 2100);
  EXPECT_TRUE(msg.subframe[0].sub2.empty());
  EXPECT_TRUE(msg.subframe[0].sub3.empty());
}

TEST(GpsdRawParser, SubframePageSelectsWithinFourAndFive)
{
  /* Subframes 4 and 5 share one pageid space -- gpsd's own comment says
   * "pageid is unique to all of subframes 4 and 5, handle as one" -- so the
   * page, not the subframe number, picks the arm. Page 56 is subframe 4's
   * ionosphere/UTC page.
   */
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = SUBFRAME_SET;
  data.subframe.subframe_num = 4;
  data.subframe.is_almanac = 0;
  data.subframe.pageid = 56;
  // WNt is the 8-bit UTC reference week number, so pick a value that fits;
  // 2100 would silently truncate to 52.
  data.subframe.sub4_18.WNt = 210;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.subframe.size(), 1u);
  ASSERT_EQ(msg.subframe[0].sub4_18.size(), 1u);
  EXPECT_EQ(msg.subframe[0].sub4_18[0].w_nt, 210);
  EXPECT_TRUE(msg.subframe[0].sub4_13.empty());
  EXPECT_TRUE(msg.subframe[0].sub5_25.empty());

  // The same page space is reached from subframe 5.
  data.subframe.subframe_num = 5;
  data.subframe.pageid = 51;            // subframe 5, page 25
  auto msg5 = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg5.subframe[0].sub5_25.size(), 1u);
  EXPECT_TRUE(msg5.subframe[0].sub4_18.empty());
}

TEST(GpsdRawParser, SubframeAlmanacGoesToSub5)
{
  // With is_almanac set, the payload is the generic almanac rather than a
  // specific page -- and gpsd keeps that in sub5.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.set = SUBFRAME_SET;
  data.subframe.subframe_num = 5;
  data.subframe.is_almanac = 1;
  data.subframe.pageid = 51;            // ignored while is_almanac is set

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.subframe.size(), 1u);
  EXPECT_EQ(msg.subframe[0].sub5.size(), 1u);
  EXPECT_TRUE(msg.subframe[0].sub5_25.empty());
}

TEST(GpsdRawParser, SubframeDeadArmIsNeverFilled)
{
  // sub4 is declared in gps.h and written by nothing in gpsd, exactly like
  // rtcm2_18..24. No subframe_num or pageid may route to it.
  for (uint8_t page : {51, 52, 55, 56, 63, 99})
  {
    gps_data_t data = gpsd_client::test::makeEmptyData();
    data.set = SUBFRAME_SET;
    data.subframe.subframe_num = 4;
    data.subframe.is_almanac = 0;
    data.subframe.pageid = page;

    auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
    ASSERT_EQ(msg.subframe.size(), 1u);
    EXPECT_TRUE(msg.subframe[0].sub4.empty()) << "pageid " << unsigned(page);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
