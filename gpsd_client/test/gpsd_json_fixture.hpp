#ifndef GPSD_CLIENT__TEST__GPSD_JSON_FIXTURE_HPP_
#define GPSD_CLIENT__TEST__GPSD_JSON_FIXTURE_HPP_

/// Library-only test harness: build gps_data_t from GPSd's own JSON.
///
/// gps.h publicly declares
///
///     extern int gps_unpack(const char *, struct gps_data_t *);
///
/// and libgps exports it in every version this package supports (3.20 through
/// 3.27.5, API 9 through 16.1). It is not a test-only side door: gps_read()
/// calls it on the socket receive buffer (libgps/libgps_sock.c), so feeding it
/// a JSON string exercises the same decode path a live client takes, without a
/// daemon, a device, a socket, or any timing.
///
/// Building fixtures this way rather than assigning gps_data_t members by hand
/// buys two things:
///
///  1. Fields land wherever *that* libgps puts them. The fix status is the
///     standing example -- GPSd 3.20 parses TPV "status" into gps_data_t.status
///     while 3.21+ parses it into gps_data_t.fix.status -- so fixtures need no
///     #if for it, unlike hand-built structs.
///  2. Version-dependent decode quirks are exercised rather than bypassed. See
///     skyJson() for one that silently drops every satellite if ignored.
///
/// Include order matters: gps.h defines STATUS_* macros that collide with the
/// ROS message constants, so gpsd_parser.hpp (which includes the message
/// headers first) must come before anything that pulls in gps.h.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <gpsd_client/gpsd_parser.hpp>

namespace gpsd_client
{
namespace test
{

/// A gps_data_t initialized the way gps_open() leaves one.
///
/// gps_open() does more than zero the struct: it clears the fix, DOP and
/// attitude sub-structs, which sets their members to NAN rather than 0.0
/// (libgps/libgps_core.c). GPSd uses NAN as its "unknown" sentinel throughout,
/// so a zero-filled gps_data_t misrepresents "unknown" as "exactly zero" and
/// makes tests agree with a parser that a live client would disagree with.
///
/// gps_clear_gst() and gps_clear_log() are not available in every supported
/// libgps, and -- unlike most differences here -- their availability does not
/// track the API version: GPSd 3.24 and 3.26.1 are both API 14.0, yet only
/// 3.26.1 declares gps_clear_gst(). They are therefore detected by CMake
/// (check_cxx_symbol_exists) rather than guarded on GPSD_API_MAJOR_VERSION.
gps_data_t makeEmptyData();

/// Fill one of GPSd's fixed char[N] members with @p value.
///
/// A handful of fields no JSON report can reach -- the device list, imu[].msg,
/// fixsource_t::spec, the VERSION and ERROR union arms -- have to be written
/// into gps_data_t by hand. This is how a fixture writes them.
///
/// Preferred over snprintf for the same reason the parser uses strnlen rather
/// than strlen: the bound comes from the member itself. N is deduced from the
/// array reference, so it cannot drift from the field being written the way a
/// hand-passed sizeof can, a pointer member is a compile error rather than a
/// silent one-word write, and there is no format string for a stray % in a
/// device path to reinterpret. Over-long values are truncated to fit; the
/// whole tail is zeroed, so what the parser reads back never depends on what
/// the array held before.
template <std::size_t N>
void setCharArray(char (&field)[N], std::string_view value)
{
  static_assert(N > 0, "a char array field always has room for a terminator");
  const std::size_t length = std::min(value.size(), N - 1);
  value.copy(field, length);
  std::fill(field + length, field + N, '\0');
}

/// Feed one or more GPSd JSON reports into @p data.
///
/// Multiple concatenated JSON objects in a single string are supported, exactly
/// as they arrive on the wire (GPSd packs several reports into one write, which
/// is why gps_unpack loops over segments).
///
/// Reports accumulate into @p data, mirroring a live session: a TPV followed by
/// a SKY leaves both the fix and the skyview populated.
///
/// Returns gps_unpack's status, but do not mistake that for validation.
/// **No released GPSd propagates parse errors out of gps_unpack()**: through
/// 3.27.5 its inner loop merely breaks on a failed segment and the function
/// returns 0 regardless (libgps/libgps_sock.c). Truncated JSON, a string where
/// a number belongs, and an unknown class all come back 0. Even the `set` mask
/// is no help -- the TPV branch assigns STATUS_SET before checking whether the
/// parse succeeded -- so libgps offers no reliable malformed-input signal at
/// all. GPSd master has since added error propagation, hence returning the
/// status rather than void: it will start being meaningful on some future
/// release.
///
/// The practical consequence is that a typo in fixture JSON fails as a
/// confusing assertion about a value, not as a parse error. That is why the
/// builders below exist -- prefer them to hand-written JSON strings, and always
/// assert on decoded values rather than on this return.
int unpack(gps_data_t& data, const std::string& json);

/// One satellite in a SKY report.
struct Satellite
{
  int prn = 0;
  double elevation = 0.0;
  double azimuth = 0.0;
  double snr = 0.0;
  bool used = false;
  /// GNSSID_GPS, GNSSID_SBAS, ... (gps.h). Numeric because the JSON is numeric.
  int gnssid = GNSSID_GPS;
  /// GPSd ignores gnssid and sigid when svid is zero, so default it to the PRN.
  int svid = -1;
};

/// Dilution-of-precision values for a SKY report. NAN means "omit the key".
struct Dop
{
  double xdop = NAN;
  double ydop = NAN;
  double hdop = NAN;
  double vdop = NAN;
  double tdop = NAN;
  double pdop = NAN;
  double gdop = NAN;
};

/// The fields of a TPV report this harness emits. NAN (the default for the
/// doubles) means "omit the key", so a fixture only states what it cares about.
///
/// Restricted to keys accepted by every supported libgps: GPSd 3.20 and 3.21
/// have no catch-all t_ignore entry in their TPV/SKY attribute tables and will
/// fail to parse an object containing a key they do not know.
struct Tpv
{
  int mode = MODE_NO_FIX;
  /// STATUS_* from gps.h, or -1 to omit. Lands in gps_data_t.status on API 9
  /// and gps_data_t.fix.status on API 10+; callers need not care which.
  int status = -1;
  /// ISO 8601, e.g. "2023-11-14T22:13:20.500Z". Empty to omit.
  std::string time;
  double latitude = NAN;
  double longitude = NAN;
  double altitude = NAN;
  double track = NAN;
  double speed = NAN;
  double climb = NAN;
  double eph = NAN;
  double epv = NAN;
  double ept = NAN;
  double epx = NAN;
  double epy = NAN;
  double epd = NAN;
  double eps = NAN;
  double epc = NAN;
};

/// Render @p tpv as a GPSd TPV report.
std::string tpvJson(const Tpv& tpv);

/// Render a GPSd SKY report.
///
/// Emits the "nSat" key only when the libgps being built against tolerates it.
/// This is not cosmetic, and the boundary is not where the version numbers
/// suggest. Once libgps reads nSat, a SKY report without it takes an early
/// return that clears SATELLITE_SET and leaves satellites_visible at 0 --
/// every satellite in the report is silently discarded. But nSat landed
/// *mid*-API-13, a day before the bump to 14, so keying on API >= 14 loses
/// every satellite when building against the end of API 13. Meanwhile GPSd
/// 3.20/3.21 reject unknown keys outright, so it cannot simply always be
/// emitted either. See the implementation for the resulting rule.
///
/// uSat is deliberately never emitted: libgps recalculates the used and visible
/// counts from the satellite array and explicitly ignores nSat/uSat for them.
std::string skyJson(const std::vector<Satellite>& satellites,
                    const Dop& dop = Dop{},
                    const std::string& time = "");

/// Convenience: an online 3D fix with three satellites, two of them used.
///
/// Mirrors the hand-built makeThreeDFix() in test_gpsd_parser.cpp so the two
/// construction paths can be compared directly.
gps_data_t makeThreeDFixFromJson();

}  // namespace test
}  // namespace gpsd_client

#endif  // GPSD_CLIENT__TEST__GPSD_JSON_FIXTURE_HPP_
