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

// GPSd renamed STATUS_FIX to STATUS_GPS in 3.23; the value (1) never changed.
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
  // GPSd uses NaN for "unknown" throughout. A raw message that reported 0.0
  // instead would be asserting a measurement that was never made.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_TRUE(std::isnan(msg.fix.latitude));
  EXPECT_TRUE(std::isnan(msg.fix.longitude));
  EXPECT_TRUE(std::isnan(msg.dop.hdop));
}

TEST(GpsdRawParser, CarriesTheSetMaskVerbatim)
{
  // The mask is copied undecoded, so a consumer can tell that GPSd
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
#ifdef HAVE_GPS_FIX_STATUS
  data.fix.status = kStatusGps;
#else
  data.status = kStatusGps;
#endif

  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
#ifdef HAVE_GPS_FIX_STATUS
  EXPECT_EQ(msg.fix.status, kStatusGps);
#else
  EXPECT_EQ(msg.status, kStatusGps);
#endif
}

// --- Sensor and device arrays -------------------------------------------------------

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

#ifdef HAVE_GPS_DATA_IMU
TEST(GpsdRawParser, ImuIsTerminatedByAnEmptyMsg)
{
  /* imu[] carries no count. GPSd's own JSON dumper walks it until
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

/* Probed by CMake, not keyed on the API version: gps_data_t::source arrived
 * *within* API 14.0 -- GPSd 3.24 does not have it, 3.25 does, and both report
 * 14.0. A version guard here compiled fine against the reference rev and broke
 * against a distro libgps. See CheckStructHasMember in CMakeLists.txt.
 */
#ifdef HAVE_GPS_DATA_SOURCE
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
#endif  // HAVE_GPS_DATA_SOURCE -- added during API 14.0, at GPSd 3.25

TEST(GpsdRawParser, AnUnterminatedCharArrayStopsAtTheEndOfTheArray)
{
  // GPSd's char[N] members carry no guarantee of a NUL: a driver that fills
  // the array exactly leaves no room for one, and GPSd's own code reads these
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

// --- Time transfer: TOFF, PPS and qErr ------------------------------------
//
// These three are reachable only from the library-only tests. GPSd's whole 196-log corpus
// contains no TOFF or PPS report, because they do not come from the receiver's
// data stream at all -- they are produced by the daemon from a PPS signal on a
// real serial line. gpsfake replays recorded device output, so it can never
// generate one. If these are not covered here they are not covered anywhere.
//
// toff, pps, qErr and qErr_time sit outside gps_data_t's report union and have
// been present since API 9, so no guard is needed. Note that GPSd's UNION_SET
// macro nonetheless lists TOFF_SET and PPS_SET, which is why the mask alone is
// not a safe guide to what is a union arm -- the struct is.

/* The TOFF tests populate gps_data_t directly rather than calling unpack(),
 * which every other test here uses.
 *
 * On GPSd 3.20 through 3.24, libgps dispatches the TOFF class to
 * json_pps_read() instead of json_toff_read(). A TOFF report decodes into
 * gps_data_t::pps, ::toff stays zeroed, and TOFF_SET goes up regardless.
 * GPSd 3.25 fixes this. See docs/gpsd-quirks.md.
 *
 * No guard expresses that boundary: 3.24 and 3.25 share API 14.0, and both
 * toff and pps exist in every supported version, so only the runtime routing
 * differs. These tests therefore assert on the fill code and skip the JSON
 * decode, which on those releases cannot deliver a TOFF to ::toff at all.
 *
 * PpsReportReachesTheMessageIncludingQErr below keeps the JSON round-trip,
 * since PPS routes correctly on every supported version.
 */

TEST(GpsdRawParser, ToffReportReachesTheMessage)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.toff.real.tv_sec = 1700000000;
  data.toff.real.tv_nsec = 250000000;
  data.toff.clock.tv_sec = 1700000000;
  data.toff.clock.tv_nsec = 250000123;
  data.set |= TOFF_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_TRUE(msg.set & gpsd_client::GpsdRawMsg::SET_TOFF);
  EXPECT_EQ(msg.toff.real.sec, 1700000000);
  EXPECT_EQ(msg.toff.real.nanosec, 250000000u);
  EXPECT_EQ(msg.toff.clock.sec, 1700000000);
  // The point of TOFF: the offset between the two clocks. The two nsec values
  // differ by 123 while the sec values match, so a real/clock mix-up shows up
  // here and nowhere else.
  EXPECT_EQ(msg.toff.clock.nanosec, 250000123u);
}

TEST(GpsdRawParser, PpsReportReachesTheMessageIncludingQErr)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::test::unpack(data,
      R"({"class":"PPS","device":"/dev/ttyS0",)"
      R"("real_sec":1700000001,"real_nsec":0,)"
      R"("clock_sec":1700000000,"clock_nsec":999999000,)"
      R"("precision":-20,"qErr":-1234})");

  ASSERT_TRUE(data.set & PPS_SET) << "libgps did not report a PPS";

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_TRUE(msg.set & gpsd_client::GpsdRawMsg::SET_PPS);
  EXPECT_EQ(msg.pps.real.sec, 1700000001);
  EXPECT_EQ(msg.pps.clock.sec, 1700000000);
  EXPECT_EQ(msg.pps.clock.nanosec, 999999000u);

  // qErr rides in on the PPS report but lives in its own member, not in pps.
  // It is signed picoseconds, so a negative value is the interesting case: a
  // narrower or unsigned field would mangle it.
  EXPECT_EQ(msg.q_err, -1234);
}

// GPSd reads PPS's "precision" and discards it -- there is a FIXME saying so
// in libgps_json.c. It reaches no struct member, so there is nothing for the
// generator to map and nothing to assert here; the message-side guarantee
// that no field exists without a gps.h member behind it lives in
// tools/test_generated_messages.py (Completeness).

TEST(GpsdRawParser, ToffAndPpsFillIndependently)
{
  // toff and pps are separate members that our fill copies one after the
  // other. A copy-paste between the two paths -- writing toff's values into
  // pps, or blanking one while filling the other -- is the failure this
  // catches, so all four values are distinct and none is zero.
  //
  // Set directly rather than unpacked, for the reason above the TOFF test.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  data.pps.real.tv_sec = 11;
  data.pps.clock.tv_sec = 22;
  data.toff.real.tv_sec = 33;
  data.toff.clock.tv_sec = 44;
  data.qErr = 7;
  data.set |= TOFF_SET | PPS_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_EQ(msg.toff.real.sec, 33);
  EXPECT_EQ(msg.toff.clock.sec, 44);
  EXPECT_EQ(msg.pps.real.sec, 11);
  EXPECT_EQ(msg.pps.clock.sec, 22);
  // qErr rides in on PPS but lives outside both timedelta_t members.
  EXPECT_EQ(msg.q_err, 7);
}

// --- Report union dispatch -----------------------------------------

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
  EXPECT_TRUE(msg.error.empty());
  EXPECT_TRUE(msg.osc.empty());
  EXPECT_TRUE(msg.raw.empty());
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
