# GPSd quirks and how `gpsd_client` handles them

GPSd and its client library have behaviours that surprise callers. This
document records the ones that shape code in this repository, so a reader
meeting one of them does not mistake it for a defect here.

Each entry states what GPSd does, then what this package does about it.

For the resulting message shapes, see
[gpsd-raw-message-structure.md](gpsd-raw-message-structure.md).

---

## An API version pair names a range of header states

**GPSd:** `GPSD_API_MAJOR_VERSION` and `GPSD_API_MINOR_VERSION` change less
often than `gps.h` does. Several releases share one pair while carrying
different structs. GPSd 3.24, 3.25 and 3.26.1 all report API 14.0, yet 3.24 has
neither `gps_data_t::source` nor the `rtcm3_4076` union arm, 3.25 has `source`
only, and 3.26.1 has both. Function availability drifts the same way: only
3.26.1 declares `gps_clear_gst()`.

**Here:** nothing tests a version number to decide whether a member exists.
The generated fill code asks C++ directly, through `if constexpr` over
`GPSD_DEFINE_HAS_MEMBER` traits. Where the preprocessor needs the answer —
compiling a whole test out — CMake probes the header with
`check_struct_has_member` or `check_cxx_symbol_exists`.

A version comparison compiles against the revision the messages came from and
breaks against a distribution's libgps. The CI sweep therefore builds GPSd 3.24
and 3.25 as well as the ten reference revisions, because those two bracket
changes that no version comparison can express.

## libgps decodes fewer report classes than GPSd emits

**GPSd:** `libgps_json.c` decodes AIS, ATT, DEVICE, DEVICES, ERROR, GST, IMU,
OSC, PPS, RAW, RTCM2, RTCM3, SKY, TOFF, TPV, VERSION and WATCH. It ignores every
other class silently. The daemon emits more than that — a plain JSON watcher
receives `SUBFRAME` reports that libgps drops.

**Here:** the `gpsd_json` topic sidesteps it. `gps_read()` copies the JSON line
into the caller's buffer *before* `gps_unpack()` decides whether it understands
the class, so every report reaches a subscriber as text even when libgps cannot
decode it into `gps_data_t`. SUBFRAME is the case that proves it: replaying
`ublox-ned-m8t-sbfrx3`, 438 of 443 forwarded lines carried SUBFRAME while
`SUBFRAME_SET` appeared on 0 typed reports.

`gps_data_t::log` still never populates, and `GPSDRaw` no longer carries
`subframe` at all — that data is reachable only as JSON. An end-to-end test
asserts both halves together: SUBFRAME present on `gpsd_json`, absent from the
typed mask. If a future libgps grows the missing readers, that test fails and
the typed message can carry them again.

An unparsed class raises no error — it simply never arrives — so only an
end-to-end test distinguishes "libgps does not decode this" from "our fill code
is broken".

## libgps decodes TOFF into the wrong member on GPSd ≤ 3.24

**GPSd:** through 3.24, the `TOFF` branch of `libgps_json_unpack()` calls
`json_pps_read()` instead of `json_toff_read()`. A TOFF report lands in
`gps_data_t::pps`, `::toff` stays zeroed, and `TOFF_SET` goes up regardless.
`json_toff_read()` compiles in and never runs. GPSd 3.25 fixes this.

**Consequences:** `toff` is unreachable through libgps on GPSd ≤ 3.24, and a
PPS report immediately followed by a TOFF loses the PPS values, because both
land in `::pps`. The mask hides it — both `PPS_SET` and `TOFF_SET` end up set.

**Here:** the TOFF tests populate `gps_data_t` directly and assert on the fill
code rather than round-tripping through libgps's JSON. The PPS tests keep the
JSON path, since PPS routes correctly everywhere.

3.24 and 3.25 share API 14.0, so no version comparison separates them, and
`toff` and `pps` exist in every supported version, so no struct probe detects
it either. Only the runtime routing differs.

## `set` keeps union bits across later reports

**GPSd:** the `TPV` handler assigns `set` outright, clearing stale bits. The
`SKY` handler only ORs its own bits in. After an RTCM3 report, `SET_RTCM3` and
the union arm behind it survive every following `SKY` report until some later
report clears them. On a receiver emitting many `SKY` reports per `TPV`, that
lasts a long time.

The tell is a combination no single report produces, such as `SET_RTCM3` and
`SET_SATELLITE` together.

**Here:** `gpsd_client` publishes `gpsd_json` per report, straight from
`gps_read`'s message argument, so each report reaches subscribers exactly once
regardless of what the mask says. `GPSDRaw` copies the mask verbatim — a topic
named "raw" that quietly repaired its input would serve subscribers worse — so
a raw subscriber testing a union bit still needs this caveat.

Non-union bits — `SET_LATLON`, `SET_ALTITUDE`, `SET_SATELLITE` and friends —
mean what you expect. The caveat applies only to the union.

## Union membership is not what the mask says, and it changes by version

**GPSd:** two independent traps here.

*The mask is not a membership list.* `UNION_SET` includes `TOFF_SET` and
`PPS_SET`, but `toff` and `pps` are plain members past the union's closing
brace. The macro over-reports.

*Membership moves.* Members leave the union between versions without changing
name or type. `attitude` is inside it through API 11.0 and outside from API
12.0; `gst` is inside through API 13.0 and outside from API 14.0.

That matters because reading a union member the report did not populate is a
read of an inactive union member — undefined behaviour, not merely a stale
value — so whether a given member needs a mask check depends on the version.

**Here:** the generator decides which members are union arms from the struct
layout at each revision, never from the mask and never from a hand-kept list.
The generated fill guards exactly those on `gps_data_t::set`, and a test
re-scans the union out of `gps.h` and fails if any published arm is copied
without its guard. See
[gpsd-raw-message-structure.md](gpsd-raw-message-structure.md) for which message
fields that makes conditional.

## `gps_read()` ignores the caller's buffer length

**GPSd:** `gps_read(gpsdata, message, message_len)` reassigns `message_len` to
the length of the JSON line it found, then `memcpy`s that many bytes into
`message`. The value the caller passed does not bound the copy.

**Here:** `gpsd_client` sizes its buffer from `GPS_JSON_RESPONSE_MAX` with
headroom, rather than trusting the length argument to protect it.

## An empty `skyview` alongside a non-zero `satellites_used`

**GPSd:** some receivers emit dilution-of-precision updates with no satellite
list. GPSd forwards these as a `SKY` report carrying `hdop`/`uSat` and no
`satellites` array. libgps then clears its skyview and drops `SATELLITE_SET`
from the mask, while parsing `uSat` into `satellites_used` (GPSd 3.26.1 and
newer). The receiver is reporting how many satellites it used without saying
which.

**Here:** the message mirrors that faithfully:

```
satellites_visible : 0
satellites_used    : 12
skyview            : []
set & SET_SATELLITE: false
```

Test the mask to learn whether a message carries a skyview:

```cpp
if (msg.set & gps_extended_msgs::msg::GPSDRaw16v1::SET_SATELLITE) {
  // skyview and satellites_visible describe this report
}
```

This depends on what the receiver sends, not on the GPSd version. On GPSd
without `uSat`, libgps returns before resetting `satellites_used`, so the count
there can be stale.

## Arrays without counts

**GPSd:** `rawdata_t::meas[]` and `gps_data_t::imu[]` carry no length. GPSd's
own code marks unused `meas[]` entries with an `svid` of 0 or 255, and
terminates `imu[]` at the first entry with an empty `msg` string.

**Here:** the parser applies GPSd's own rules — skipping `meas[]` entries by
`svid`, which filters rather than terminates, and trimming `imu[]` at the first
empty `msg`.

## `gps.h` macros collide with ROS message constants

**GPSd:** `gps.h` defines the report mask and status values as preprocessor
macros — `LATLON_SET`, `STATUS_FIX`. The C preprocessor replaces any identifier
of the same name, including generated ROS message constants, wherever both
headers appear.

**Here:** the generated constants reverse the name, so GPSd's `LATLON_SET`
becomes `SET_LATLON`. A test includes `<gps.h>` before the generated headers to
keep that guarantee standing.

GPSd also renames these between releases: `STATUS_FIX` and `STATUS_DGPS_FIX`
became `STATUS_GPS` and `STATUS_DGPS`. Code that needs them probes with `#ifdef`
rather than comparing versions.

## `NaN` means unknown

**GPSd:** GPSd writes `NaN` into floating-point members it has no value for.

**Here:** the fill code copies `NaN` through rather than substituting `0.0`, and
tests pin that. A subscriber must treat `NaN` as "unknown" instead of assuming a
number.

## GPSd reports a fix status after the fix goes stale

**GPSd:** GPSd keeps reporting status OK once it has seen a fix, even after the
current solution goes away.

**Here:** the `check_fix_by_variance` parameter discards fixes whose `epx`,
`epy` or `epv` are not finite, which rejects those stale results. It stays off
by default and never gates the raw topic — filtering there would make "raw"
a misnomer.

## `GPSD_API_MAJOR_VERSION` skips 15

**GPSd:** the major version goes from 14 to 16. No release ever carried 15.

**Here:** ten API pairs exist and no `GPSDRaw15v0` type does.
