#include "gpsd_json_fixture.hpp"

#include <charconv>
#include <cmath>
#include <sstream>
#include <string_view>
#include <system_error>
#include <vector>

namespace gpsd_client
{
namespace test
{
namespace
{

/// Append "key":value, but only when the value is known (not NAN).
///
/// std::to_chars emits the shortest decimal that reads back as the identical
/// double, so a fixture's constants survive the trip through JSON and can be
/// compared with EXPECT_DOUBLE_EQ. Preferred over a printf conversion: there
/// is no format string to disagree with the argument type, no silent
/// truncation, and -- unlike both printf and operator<<, which follow the
/// stream's locale -- no chance of a comma decimal separator turning the
/// fixture into invalid JSON.
void appendReal(std::ostringstream& out, const char* key, double value)
{
  if (!std::isfinite(value))
  {
    return;
  }
  char buf[32];
  const std::to_chars_result result = std::to_chars(buf, buf + sizeof(buf), value);
  if (result.ec != std::errc())
  {
    // Unreachable: a double's shortest round-trip form needs at most 24 bytes.
    // Emitting nothing beats emitting whatever the buffer happens to hold.
    return;
  }
  out << ",\"" << key << "\":" << std::string_view(buf, result.ptr - buf);
}

}  // namespace

gps_data_t makeEmptyData()
{
  gps_data_t data{};

  // Mirrors gps_open()'s tail in libgps/libgps_core.c.
  data.set = 0;
  data.satellites_used = 0;
  gps_clear_att(&data.attitude);
  gps_clear_dop(&data.dop);
  gps_clear_fix(&data.fix);
#ifdef HAVE_GPS_CLEAR_GST
  gps_clear_gst(&data.gst);
#endif
#ifdef HAVE_GPS_CLEAR_LOG
  gps_clear_log(&data.log);
#endif

  return data;
}

int unpack(gps_data_t& data, const std::string& json)
{
  /* gps_unpack() took a mutable char* through GPSd 3.24 and a const char*
   * from 3.25 on. Both are API 14.0, so the difference cannot be keyed on
   * GPSD_API_MAJOR_VERSION -- and const_cast'ing away the older signature
   * would be a bet that no libgps in the supported range writes through the
   * pointer. Copying into a mutable buffer satisfies both declarations and
   * needs no bet.
   */
  std::vector<char> buffer(json.begin(), json.end());
  buffer.push_back('\0');
  return gps_unpack(buffer.data(), &data);
}

std::string tpvJson(const Tpv& tpv)
{
  std::ostringstream out;
  out << "{\"class\":\"TPV\",\"device\":\"/dev/ttyS0\"";
  out << ",\"mode\":" << tpv.mode;

  if (tpv.status >= 0)
  {
    out << ",\"status\":" << tpv.status;
  }
  if (!tpv.time.empty())
  {
    out << ",\"time\":\"" << tpv.time << "\"";
  }

  appendReal(out, "lat", tpv.latitude);
  appendReal(out, "lon", tpv.longitude);
  appendReal(out, "alt", tpv.altitude);
  appendReal(out, "track", tpv.track);
  appendReal(out, "speed", tpv.speed);
  appendReal(out, "climb", tpv.climb);
  appendReal(out, "eph", tpv.eph);
  appendReal(out, "epv", tpv.epv);
  appendReal(out, "ept", tpv.ept);
  appendReal(out, "epx", tpv.epx);
  appendReal(out, "epy", tpv.epy);
  appendReal(out, "epd", tpv.epd);
  appendReal(out, "eps", tpv.eps);
  appendReal(out, "epc", tpv.epc);

  out << "}";
  return out.str();
}

std::string skyJson(const std::vector<Satellite>& satellites,
                    const Dop& dop,
                    const std::string& time)
{
  std::ostringstream out;
  out << "{\"class\":\"SKY\",\"device\":\"/dev/ttyS0\"";

  if (!time.empty())
  {
    out << ",\"time\":\"" << time << "\"";
  }

  appendReal(out, "xdop", dop.xdop);
  appendReal(out, "ydop", dop.ydop);
  appendReal(out, "hdop", dop.hdop);
  appendReal(out, "vdop", dop.vdop);
  appendReal(out, "tdop", dop.tdop);
  appendReal(out, "pdop", dop.pdop);
  appendReal(out, "gdop", dop.gdop);

  /* Emitted from API 11 on. The threshold is about what libgps *tolerates*,
   * not when nSat became meaningful, because the two do not line up with
   * version bumps:
   *
   *   - GPSd 3.20/3.21 (API 9, 10) have no catch-all t_ignore in their SKY
   *     attribute table and reject the unknown key outright, losing the whole
   *     report. So nSat must be omitted there -- those versions count the
   *     satellites array themselves.
   *   - GPSd 3.22 (API 11) added t_ignore, so from there on an unrecognised
   *     nSat is harmlessly skipped.
   *   - nSat itself landed mid-API-13, one day before the bump to 14, and
   *     from then on its absence makes libgps discard every satellite.
   *
   * Keying on API >= 11 therefore covers both halves of API 13 and every
   * other in-between state, which a threshold at 14 did not.
   */
#if GPSD_API_MAJOR_VERSION >= 11
  out << ",\"nSat\":" << satellites.size();
#endif

  out << ",\"satellites\":[";
  for (size_t i = 0; i < satellites.size(); ++i)
  {
    const Satellite& sat = satellites[i];
    if (i > 0)
    {
      out << ",";
    }
    out << "{\"PRN\":" << sat.prn
        << ",\"el\":" << sat.elevation
        << ",\"az\":" << sat.azimuth
        << ",\"ss\":" << sat.snr
        << ",\"used\":" << (sat.used ? "true" : "false")
        << ",\"gnssid\":" << sat.gnssid
        << ",\"svid\":" << (sat.svid >= 0 ? sat.svid : sat.prn)
        << "}";
  }
  out << "]}";

  return out.str();
}

gps_data_t makeThreeDFixFromJson()
{
  Tpv tpv;
  tpv.mode = MODE_3D;
  tpv.time = "2023-11-14T22:13:20.500Z";  // 1700000000.5
  tpv.latitude = 29.44;
  tpv.longitude = -98.61;
  tpv.altitude = 250.0;
  tpv.track = 90.0;
  tpv.speed = 2.5;
  tpv.climb = 0.25;
  tpv.epx = 1.5;
  tpv.epy = 2.5;
  tpv.epv = 3.5;
  tpv.epd = 0.5;
  tpv.eps = 0.75;
  tpv.epc = 1.25;
  tpv.ept = 0.005;
  tpv.eph = 4.5;

  Dop dop;
  dop.pdop = 1.1;
  dop.hdop = 1.2;
  dop.vdop = 1.3;
  dop.tdop = 1.4;
  dop.gdop = 1.5;

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

  gps_data_t data = makeEmptyData();
  unpack(data, tpvJson(tpv) + skyJson(satellites, dop));

  // GPSd only reports online-ness on the socket, never in a report body, so a
  // JSON-built fixture has to say so explicitly.
  data.online.tv_sec = 100;

  return data;
}

}  // namespace test
}  // namespace gpsd_client
