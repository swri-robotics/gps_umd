/// Tests for the raw parser and the generated fill code behind it.
///
/// Normal include order here (gpsd_client headers first, which pull gps.h in
/// behind the message headers). The gps.h-first case has its own translation
/// unit in test_gpsd_raw_include_order.cpp, because it cannot include anything
/// that reaches the legacy GPSFix/GPSStatus messages -- those still collide
/// with gps.h and always have.

#include <gtest/gtest.h>

#include <cmath>
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

  EXPECT_DOUBLE_EQ(msg.fix_latitude, 29.44);
  EXPECT_DOUBLE_EQ(msg.fix_longitude, -98.61);
  EXPECT_DOUBLE_EQ(msg.fix_altitude, 250.0);
  EXPECT_DOUBLE_EQ(msg.fix_speed, 2.5);
  EXPECT_EQ(msg.fix_mode, MODE_3D);

  EXPECT_DOUBLE_EQ(msg.dop_hdop, 1.2);
  EXPECT_DOUBLE_EQ(msg.dop_pdop, 1.1);
  EXPECT_DOUBLE_EQ(msg.dop_gdop, 1.5);

  // timespec_t -> builtin_interfaces/Time keeps the nanoseconds.
  EXPECT_EQ(msg.fix_time.sec, 1700000000);
  EXPECT_EQ(msg.fix_time.nanosec, 500000000u);
  EXPECT_EQ(msg.online.sec, 100);
}

TEST(GpsdRawParser, SkyviewIsTruncatedToTheValidCount)
{
  // The array the generator deliberately leaves alone: its length lives in a
  // sibling field, so publishing the whole fixed array would emit MAXCHANNELS
  // entries of uninitialised satellites.
  gps_data_t data = gpsd_client::test::makeThreeDFixFromJson();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(42, 0));

  ASSERT_EQ(msg.skyview_prn.size(), 3u);
  EXPECT_LT(msg.skyview_prn.size(), static_cast<std::size_t>(MAXCHANNELS));

  EXPECT_EQ(msg.skyview_prn[0], 10);
  EXPECT_DOUBLE_EQ(msg.skyview_elevation[0], 30.0);
  EXPECT_DOUBLE_EQ(msg.skyview_azimuth[0], 100.0);
  EXPECT_DOUBLE_EQ(msg.skyview_ss[0], 40.0);
  EXPECT_TRUE(msg.skyview_used[0]);
  EXPECT_EQ(msg.skyview_prn[2], 12);
  EXPECT_FALSE(msg.skyview_used[2]);
}

TEST(GpsdRawParser, SkyviewCountIsClampedAgainstGarbage)
{
  // satellites_visible is a plain int; a truncated or stale report can leave
  // it negative or past the end of the array, and neither may be trusted as a
  // loop bound.
  gps_data_t data = gpsd_client::test::makeEmptyData();

  data.satellites_visible = -1;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).skyview_prn.size(), 0u);

  data.satellites_visible = MAXCHANNELS + 500;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).skyview_prn.size(),
            static_cast<std::size_t>(MAXCHANNELS));
}

TEST(GpsdRawParser, PreservesNanRatherThanZeroing)
{
  // GPSd uses NaN for "unknown" throughout. A raw message that reported 0.0
  // instead would be asserting a measurement that was never made.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::GpsdRawMsg msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_TRUE(std::isnan(msg.fix_latitude));
  EXPECT_TRUE(std::isnan(msg.fix_longitude));
  EXPECT_TRUE(std::isnan(msg.dop_hdop));
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
  EXPECT_EQ(msg.fix_status, kStatusGps);
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
  gpsd_client::test::setCharArray(data.devices.list[0].path, "/dev/ttyS0");
  gpsd_client::test::setCharArray(data.devices.list[1].path, "/dev/ttyS1");

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.devices_list_path.size(), 2u);
  EXPECT_LT(msg.devices_list_path.size(), capacity);
  EXPECT_EQ(msg.devices_list_path[0], "/dev/ttyS0");
  EXPECT_EQ(msg.devices_list_path[1], "/dev/ttyS1");
  EXPECT_EQ(msg.devices_ndevices, 2);

  // Same garbage-clamping as skyview: ndevices is a plain int.
  data.devices.ndevices = -1;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).devices_list_path.size(), 0u);
  data.devices.ndevices = static_cast<int>(capacity) + 100;
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).devices_list_path.size(),
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
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).imu_msg.size(), 0u);

  gpsd_client::test::setCharArray(data.imu[0].msg, "UBX-ESF-RAW");
  gpsd_client::test::setCharArray(data.imu[1].msg, "UBX-ESF-RAW");
  data.imu[1].gyro_x = 1.5;
  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  ASSERT_EQ(msg.imu_msg.size(), 2u);
  EXPECT_EQ(msg.imu_msg[0], "UBX-ESF-RAW");
  EXPECT_DOUBLE_EQ(msg.imu_gyro_x[1], 1.5);

  // A gap terminates: entry 3 is stamped but unreachable past the empty 2.
  gpsd_client::test::setCharArray(data.imu[3].msg, "UBX-ESF-RAW");
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).imu_msg.size(), 2u);

  // All ten stamped: bounded by the array, never past it.
  for (std::size_t i = 0; i < max_imu; ++i)
  {
    gpsd_client::test::setCharArray(data.imu[i].msg, "UBX-ESF-RAW");
  }
  EXPECT_EQ(makeParser()->parseRaw(data, rclcpp::Time(0, 0)).imu_msg.size(), max_imu);
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
  gpsd_client::test::setCharArray(data.source.spec, "localhost:2947");

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.source_spec, "localhost:2947");
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

  EXPECT_EQ(sizeof(data.dev.path), msg.dev_path.size());
  EXPECT_EQ(std::string(sizeof(data.dev.path), 'x'), msg.dev_path);
}

TEST(GpsdRawParser, ATerminatedCharArrayStopsAtTheTerminator)
{
  // The other half of the pair: bounding by sizeof must not also mean
  // publishing the padding after a short, properly terminated string.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::test::setCharArray(data.dev.path, "/dev/ttyS0");

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  EXPECT_EQ("/dev/ttyS0", msg.dev_path);
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
  EXPECT_EQ(msg.toff_real.sec, 1700000000);
  EXPECT_EQ(msg.toff_real.nanosec, 250000000u);
  EXPECT_EQ(msg.toff_clock.sec, 1700000000);
  // The point of TOFF: the offset between the two clocks. The two nsec values
  // differ by 123 while the sec values match, so a real/clock mix-up shows up
  // here and nowhere else.
  EXPECT_EQ(msg.toff_clock.nanosec, 250000123u);
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
  EXPECT_EQ(msg.pps_real.sec, 1700000001);
  EXPECT_EQ(msg.pps_clock.sec, 1700000000);
  EXPECT_EQ(msg.pps_clock.nanosec, 999999000u);

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

  EXPECT_EQ(msg.toff_real.sec, 33);
  EXPECT_EQ(msg.toff_clock.sec, 44);
  EXPECT_EQ(msg.pps_real.sec, 11);
  EXPECT_EQ(msg.pps_clock.sec, 22);
  // qErr rides in on PPS but lives outside both timedelta_t members.
  EXPECT_EQ(msg.q_err, 7);
}

// --- Report union dispatch -----------------------------------------

TEST(GpsdRawParser, ReportUnionFillsOnlyTheArmTheMaskNames)
{
  /* gps_data_t packs its report arms into a union, so only the arm the set
   * mask names may be read -- touching another is a read of an inactive union
   * member. The arms are plain scalars in the message, so the mask is what
   * tells a subscriber whether they mean anything; this asserts the parser
   * honours it rather than copying whatever the union happened to hold.
   */
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::test::setCharArray(data.version.release, "3.27.5");
  data.set = DEVICE_SET | VERSION_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.version_release, "3.27.5") << "VERSION_SET was set";

  // The other arms share that storage and were not named by the mask.
  EXPECT_TRUE(msg.error.empty());
  EXPECT_FALSE(msg.osc_running);
  EXPECT_EQ(msg.osc_delta, 0);
}

TEST(GpsdRawParser, UnionArmsStayAtTheirDefaultsWithoutTheirBit)
{
  // The converse: the union holds a VERSION report, but the mask does not say
  // so, so nothing may be copied out of it.
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::test::setCharArray(data.version.release, "3.27.5");
  data.set = LATLON_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_TRUE(msg.version_release.empty())
      << "copied a union arm the mask did not name";
}

TEST(GpsdRawParser, ErrorArmIsAStringFromTheMask)
{
  gps_data_t data = gpsd_client::test::makeEmptyData();
  gpsd_client::test::setCharArray(data.error, "no such device");
  data.set = ERROR_SET;

  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.error, "no such device");
}

TEST(GpsdRawParser, ParallelArraysInAGroupAgreeInLength)
{
  /* Flattening turns one array of structs into many arrays of scalars, and
   * nothing in the type system keeps them the same length any more. The
   * generated filler writes each group from a single loop, so this is a
   * backstop against that guarantee being lost.
   */
  gps_data_t data = gpsd_client::test::makeThreeDFixFromJson();
  auto msg = makeParser()->parseRaw(data, rclcpp::Time(0, 0));

  const std::size_t n = msg.skyview_prn.size();
  EXPECT_EQ(msg.skyview_ss.size(), n);
  EXPECT_EQ(msg.skyview_elevation.size(), n);
  EXPECT_EQ(msg.skyview_azimuth.size(), n);
  EXPECT_EQ(msg.skyview_used.size(), n);
  EXPECT_EQ(msg.skyview_gnssid.size(), n);

  const std::size_t d = msg.devices_list_path.size();
  EXPECT_EQ(msg.devices_list_driver.size(), d);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
