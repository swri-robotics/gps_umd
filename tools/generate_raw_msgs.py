#!/usr/bin/env python3
"""Generate the GPSDRaw<MAJOR>v<MINOR> messages and their parsers from gps.h.

Ground truth is gpsd's ``include/gps.h`` at a pinned commit per API pair (see
REFERENCE_REVS). Nothing here reads the build host's installed libgps: the
messages live in ``gps_msgs``, which is a pure interface package released to the
ROS build farm and must never gain a libgps dependency. Regenerating is a
deliberate, reviewed act, and CI runs ``--check`` so the checked-in output
cannot drift from this script.

Design decisions this implements live in docs/gpsd-raw-messages-plan.md; the
ones that constrain the code most are D1 (no libgps in gps_msgs), D2 (generated,
not hand-written), D3 (version-suffixed sub-messages), D5 (union arms selected
by the ``set`` mask), D9 (``SET_<NAME>`` constants), D10 (no AIS) and D11
(absent members detected in C++).


Why a commit and not an API version
-----------------------------------

An API pair names a *range* of header states, not one state. gpsd bumps the
version when a change begins and then keeps adding under the same number until
the next bump. Both of these are real:

  * API 14.0 covers gpsd 3.24 through 3.26.1, and ``gps_fix_t`` gains
    ant_stat, clockbias, clockdrift, jam, temp and wtemp across that range.
  * API 16.1 covers 3.27.5 (MAXCHANNELS 184) and master (230).

So a message generated from a pair's last rev is a *superset* of what some
libgps reporting that same pair provides, and generated parser code must guard
every field with the C++ detection idiom rather than assume presence (D11).

Do not trust the changelog comment block at the top of gps.h for values. It
claims MAXCHANNELS went to 185 and then 230; no release ever shipped either.
Read the ``#define``.


Type mapping
------------

Applied uniformly so generated output can be checked against a stated rule.

  gpsd C type              ROS 2 field                      Note
  -----------------------  -------------------------------  --------------------
  double                   float64
  float                    float32
  int / unsigned           int32 / uint32
  short / unsigned short   int16 / uint16
  long / unsigned long     int64 / uint64                   width is ABI
                                                            dependent; widen
  bool                     bool
  char name[N]             string                           NUL-terminated,
                                                            truncate at first NUL
  char data[N] (binary)    uint8[]                          not a string
  timespec_t               builtin_interfaces/Time          tv_sec/tv_nsec map
                                                            directly
  time_t                   int64                            seconds; NOT a ROS
                                                            Time -- may be a
                                                            duration or a TOW
  gps_mask_t               uint64                           verbatim, undecoded
  gnssid_t (API 16+)       uint8                            was unsigned char
  struct X                 GPSD<X><MAJOR>v<MINOR>           D3
  struct X arr[N]          GPSD<X><MAJOR>v<MINOR>[]         unbounded, D4
  C union                  mask + all arms, one populated   D5
  NaN sentinel             float64 NaN preserved            gpsd uses NaN for
                                                            "unknown"; never zero

Fixed C arrays become unbounded ROS arrays truncated to the valid count
(satellites_visible, devices.ndevices, ...). MAXCHANNELS is deliberately not
encoded in any message type: it is 140 or 184 depending on the rev, and it is
not a function of the API pair, so baking it in would be ambiguous as well as
wasteful.

The ``set`` bitmask constants are emitted as uint64 message constants, renamed
``<NAME>_SET`` -> ``SET_<NAME>`` (D9). The rename is forced: rosidl emits
constants as ``static constexpr`` members, gps.h defines the gpsd spellings as
global macros, and a member named STATUS_SET in a header parsed after gps.h is
destroyed by the preprocessor. gps.h defines no SET_* macros, so the flipped
form is collision-free. ``UNION_SET`` becomes ``SET_UNION`` and keeps AIS_SET in
its composite, matching gps.h.


Exclusions
----------

The single reviewable list of what is deliberately not published. Everything
else in gps_data_t must be mapped or the header-audit test fails the build.
"""

# Members of gps_data_t that are deliberately never published.
#
# ais       -- D10. struct ais_t is a ~two-dozen-arm tagged union of marine
#              vessel traffic, the largest and least relevant thing in gps.h.
#              SET_AIS is still emitted and `set` still carries the bit, so an
#              omitted AIS report stays detectable by consumers.
# gps_fd    -- process-local file descriptor, not reported data.
# update_fd -- function pointer.
# privdata  -- libgps internal state; gps.h says clients must not touch it.
# set_pending -- deferred-send bookkeeping internal to libgps.
EXCLUDED_MEMBERS = (
    "ais",
    "gps_fd",
    "update_fd",
    "privdata",
    "set_pending",
)

# API (major, minor) -> the gpsd revision the message is generated from.
#
# Released pairs use the *last* release carrying that pair, so the message is
# the superset of what any libgps reporting the pair provides (see D11 for how
# parsers cope with the older members being absent).
#
# 9.1, 10.1 and 13.0 shipped in no release at all; they are pinned to
# `<next version bump>^`, the last commit at which the pair was current, and
# each has been verified to report the pair named here.
#
# There is deliberately no 15.x entry: GPSD_API_MAJOR_VERSION jumped 14 -> 16 in
# a single commit and was never 15, even though gps.h has a "15" changelog
# stanza. Those entries describe what a build reports as API 16.
#
# gpsd 3.27, 3.27.1 and 3.27.2 are out of scope (they shipped
# api_version_major = 0 in SConscript). Nothing is lost: all three are API 16.0,
# represented here by 3.27.3.
REFERENCE_REVS = {
    (9, 0): "release-3.20",
    (9, 1): "e5279ef52",       # 2020-03-19, last commit at API 9.1
    (10, 0): "release-3.21",
    (10, 1): "42f816d59",      # 2020-08-21, last commit at API 10.1
    (11, 0): "release-3.22",
    (12, 0): "release-3.23.1",
    (13, 0): "264e808c6",      # 2022-04-06, last commit at API 13.0
    (14, 0): "release-3.26.1",
    (16, 0): "release-3.27.3",
    (16, 1): "release-3.27.5",
}

# MAXCHANNELS at each reference rev, recorded only as a cross-check that the
# rev was read correctly -- it is never emitted into a message (D4).
EXPECTED_MAXCHANNELS = {
    (9, 0): 140,
    (9, 1): 140,
    (10, 0): 140,
    (10, 1): 140,
    (11, 0): 140,
    (12, 0): 140,
    (13, 0): 140,
    (14, 0): 184,
    (16, 0): 184,
    (16, 1): 184,
}


def main() -> int:
    raise SystemExit(
        "not implemented yet -- phase 1. This file currently carries the "
        "generator's contract: the type mapping, the exclusion list, and the "
        "pinned reference revisions."
    )


if __name__ == "__main__":
    main()
