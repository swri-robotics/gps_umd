/// Proves the generated raw messages survive being parsed after gps.h.
///
/// This is the whole point of the constant renaming, and it needs its own
/// translation unit to mean
/// anything. gps.h defines every mask bit (`LATLON_SET`), the fix statuses
/// (`STATUS_RTK_FIX`) and `SET_HIGH_BIT` as plain object-like macros, while
/// rosidl emits message constants as `static constexpr` members. A generated
/// constant sharing a name with a gps.h macro is silently mangled by the
/// preprocessor in any file that saw gps.h first -- and downstream users
/// control neither our naming nor their own include order.
///
/// Two things this deliberately does NOT include:
///
///   * gpsd_parser.hpp / gpsd_raw_parser.hpp, which pull in the legacy
///     GPSFix/GPSStatus messages. Those still collide: GPSStatus defines
///     STATUS_RTK_FIX = 19 and gps.h defines STATUS_RTK_FIX = 3. That is a
///     pre-existing conflict in the released gps_msgs package, not something
///     this work introduced or can fix without changing published constants,
///     and it is why gpsd_parser.hpp documents a mandatory include order.
///   * any message header by name, so this keeps working as the selected
///     version changes.
///
/// So the claim under test is precisely: the *generated* GPSDRaw family is
/// order-independent, even though its neighbours are not.

#include <gps.h>

#include <gpsd_client/gpsd_raw_message.hpp>

#include <gtest/gtest.h>

TEST(GpsdRawIncludeOrder, GeneratedConstantsSurviveGpsHFirst)
{
  // Reaching this line at all is most of the test: if any generated constant
  // collided with a gps.h macro, this file would not compile.
  EXPECT_EQ(gpsd_client::GpsdRawMsg::SET_LATLON, static_cast<uint64_t>(LATLON_SET));
  EXPECT_EQ(gpsd_client::GpsdRawMsg::SET_ONLINE, static_cast<uint64_t>(ONLINE_SET));
  EXPECT_EQ(gpsd_client::GpsdRawMsg::SET_AIS, static_cast<uint64_t>(AIS_SET));
  // Superset, not equality, for the same mid-pair reason as above.
  EXPECT_EQ(gpsd_client::GpsdRawMsg::SET_UNION & static_cast<uint64_t>(UNION_SET),
            static_cast<uint64_t>(UNION_SET));

  // SET_HIGHEST_BIT is a *count*, not a bit, and it is the one constant that
  // legitimately disagrees with the build's gps.h. Messages are generated from
  // the last rev of an API pair, and a pair spans a range of header states:
  // GPSDRaw14v0 comes from gpsd 3.26.1 (SET_HIGH_BIT 45) but gpsd 3.24 reports
  // the same API 14.0 with 44, because EOF_SET landed mid-pair. So the message
  // can only ever know about at least as many bits as this build.
  EXPECT_GE(gpsd_client::GpsdRawMsg::SET_HIGHEST_BIT,
            static_cast<uint64_t>(SET_HIGH_BIT));
}

TEST(GpsdRawIncludeOrder, SelectedPairMatchesTheBuild)
{
  EXPECT_EQ(GPSD_RAW_FILL_MAJOR, GPSD_API_MAJOR_VERSION);
  EXPECT_EQ(GPSD_RAW_FILL_MINOR, GPSD_API_MINOR_VERSION);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
