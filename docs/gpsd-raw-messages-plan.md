# Plan: per-API-version `GPSDRaw<MAJOR>v<MINOR>` messages

Branch: `per_api_version_messages`

Goal: expose everything gpsd reports, losslessly, as versioned ROS 2 messages —
one message type per gpsd C API `(MAJOR, MINOR)` pair from API 9 to the latest —
with a matching parser in `gpsd_client` that optionally publishes it, selected at
compile time by the libgps headers the workspace is built against.

Status legend: `[ ]` not started · `[~]` in progress · `[x]` done · `[-]` dropped (with reason)

---

## 1. Established facts

These were read out of the gpsd git history (`.gpsd_versions/gpsd`, upstream
`https://gitlab.com/gpsd/gpsd.git`) rather than assumed. Re-verify if the
upstream history is ever rewritten.

### 1.1 API version → gpsd release

`GPSD_API_MAJOR_VERSION` / `GPSD_API_MINOR_VERSION` live in `include/gps.h`
(at repo root as `gps.h` before gpsd 3.22). The per-version changelog comment
block at the top of that file documents what each bump added.

| API pair | gpsd release(s) shipping it | Bump commit | Notes |
|---|---|---|---|
| 9.0 | 3.20 | `e2c26993` (2019-07-05) | Oldest version `gpsd_client` supports |
| 9.1 | **none** | `8da63ed3` (2020-01-11) | Adds `gps_data_t::leap_seconds` |
| 10.0 | 3.21 | `29991d6f` (2020-03-23) | `status` moves `gps_data_t` → `gps_fix_t` |
| 10.1 | **none** | `7c7de250` (2020-08-14) | `sub4_18` type fixes (subframe only) |
| 11.0 | 3.22 | `84640555` (2020-08-21) | |
| 12.0 | 3.23, 3.23.1 | `72054c36` (2021-03-31) | `orbit_t`, IMU array, attitude leaves the union |
| 13.0 | **none** | `81115741` (2021-11-23) | `rtcm3_msm`, `baseline_t` |
| 14.0 | 3.24, 3.25, 3.26, 3.26.1 | `0df348fe` (2022-04-07) | `MAXCHANNELS` 140 → 184, mid-pair; `gps_fix_t` also grows mid-pair (1.7) |
| 15.x | **never existed** | — | See 1.2 |
| 16.0 | 3.27, 3.27.1, 3.27.2, 3.27.3 | `bd8f7106` (2025-10-29) | SPARTN, `gnssid_t`. (`MAXCHANNELS` stays 184; the changelog's 230 is master-only.) Use **3.27.3**; the earlier three are out of scope (see 1.6) |
| 16.1 | 3.27.5 | `26951b6f` (2026-01-08) | No documented struct delta; see 1.3 |

### 1.2 API 15 was never a real value

Commit `bd8f7106` jumped `GPSD_API_MAJOR_VERSION` straight from 14 to 16 in a
single step while *also* adding a "15" stanza to the changelog comment
(`include/gps.h:114-119`). No commit ever defined the macro as 15, so no libgps
can report it.

**Decision:** do not create `GPSDRaw15v0.msg`. The API-15 changelog entries
(sigid decoding, Teseo antenna status, `errEllipse*`, `ant_power`) are part of
what a build reports as API 16 and belong in `GPSDRaw16v0.msg`.

### 1.3 Minor bumps do not always change the struct layout

- 9.1 adds a real field (`leap_seconds`).
- 10.1 only fixes `subframe_t` member types.
- 16.1 has no changelog stanza at all — the bump accompanied realigning the JSON
  protocol version with the C API version, not a header change.

**Decision:** still emit a message per `(MAJOR, MINOR)` pair, as specified, even
when two adjacent messages are byte-identical in content. The naming scheme is
the contract; identical content is acceptable and the generator will produce it
naturally.

### 1.4 Unreleased API pairs

9.1, 10.1 and 13.0 exist only as commit ranges — no `release-*` tag carries
them. They are still reachable if someone builds against a gpsd git checkout, so
messages are generated for them. Testing them requires pinning a commit SHA
rather than a tag (see phase 6).

### 1.5 The `set` mask bit assignments are append-only

Every `*_SET` bit in `gps_data_t::set` holds the same position across the whole
API 9 → 16.1 range. Diffing the mask defines between `release-3.20` and
`release-3.27.5` shows only additions at the top end, no renumbering and no
removals:

| Reference rev | `*_SET` bits | `SET_HIGH_BIT` |
|---|---|---|
| 3.20 / `8da63ed3` (API 9.0, 9.1) | 41 | 42 |
| 3.21 / `7c7de250` / 3.22 (API 10.0, 10.1, 11.0) | 42 | 43 |
| 3.23.1 / `81115741` (API 12.0, 13.0) | 43 | 44 |
| 3.26.1 (API 14.0) | 44 | 45 |
| 3.27.3 / 3.27.5 (API 16.0, 16.1) | 45 | 46 |

Added along the way: `LOG_SET` (42), `IMU_SET` (43), `EOF_SET` (44),
`SPARTN_SET` (45).

This matters for D9: a consumer that tests `SET_LATLON` gets the same value from
every `GPSDRaw*` message, so mask-testing code is portable across versions even
though the message types are not. Newer messages simply define more constants.

### 1.6 Unrelated trap: `proto_major` is not the C API version

`gpsd` also has a JSON protocol version (`api_version_major`/`_minor` in
`SConscript`, surfaced as `proto_major`/`proto_minor` in the `VERSION` report and
in `version_t`). It tracked `3.x` until gpsd 3.27, then:

| Release | reported `proto_major.proto_minor` |
|---|---|
| 3.22 – 3.23.1 | 3.14 |
| 3.24 – 3.26.1 | 3.15 |
| 3.27, 3.27.1, 3.27.2 | **0.16** — out of scope, see below |
| 3.27.3 | 3.16 |
| 3.27.5+ | 16.1 (now tracks the C API version) |

The point to carry forward is only the disambiguation: this project keys
everything off `GPSD_API_MAJOR_VERSION`/`_MINOR_VERSION` from `gps.h`, never off
`proto_major`/`proto_minor`. They are different numbers with different histories.

**Out of scope: releases reporting `api_version_major == 0`.** gpsd 3.27, 3.27.1
and 3.27.2 shipped `api_version_major = 0` in `SConscript` (fixed in 3.27.3).
Those three releases are not targeted, tested, or worked around anywhere in this
plan. Nothing is lost by skipping them: all three carry C API 16.0, which is
covered by `release-3.27.3` — the reference rev already chosen for
`GPSDRaw16v0.msg` (section 3) and for the CI matrix (phase 6). If a raw message
built against one of those releases carries `proto_major == 0` in its
`version_t` union arm, that is simply what gpsd reported; the parser does not
special-case it.

### 1.7 libgps changes things without bumping the API version

Established while building the tier-1 harness, and the single most important
correction to this plan's assumptions: **`GPSD_API_MAJOR_VERSION` is not a
complete description of libgps' source interface.** Three confirmed cases, all
*within* a single API version:

| Change | Before | After | API version |
|---|---|---|---|
| `gps_unpack()` argument | `char *` (≤ 3.24) | `const char *` (3.25+) | 14.0 **both sides** |
| `gps_clear_gst()` declared | absent (≤ 3.24) | present (3.26.1+) | 14.0 **both sides** |
| SKY `nSat` key required | absent (≤ 3.23.1) | required (3.24+) | 12.0 → 14.0 |
| **`gps_fix_t` gains 6 members** | 3.24 | 3.26.1 | 14.0 **both sides** |
| **`MAXCHANNELS`** | 140 (≤ 3.25) | 184 (3.26+) | 14.0 **both sides** |
| **`MAXCHANNELS`** | 184 (3.27.5) | 230 (master) | 16.1 **both sides** |

So a compile-time `#if GPSD_API_MAJOR_VERSION` ladder is necessary but not
sufficient. Anything that touches a *function signature* or a *declaration's
existence* must be probed by CMake (`check_cxx_symbol_exists`) or written so
that it compiles either way — see `unpack()` in
[gpsd_client/test/gpsd_json_fixture.cpp](../gpsd_client/test/gpsd_json_fixture.cpp),
which copies into a mutable buffer rather than betting on one signature.

**The last three rows break a premise this plan was built on.** Struct members
do *not* track the API version either. gpsd bumps the version when a change
*begins* and then keeps adding under the same number until the next bump, so an
API pair names a **range** of header states, not one state:

- API 14.0 spans 3.24 → 3.26.1, and `gps_fix_t` gains `ant_stat`, `clockbias`,
  `clockdrift`, `jam`, `temp` and `wtemp` across that range. (All six *are*
  listed in the gps.h API-14 changelog stanza — they simply landed after 3.24
  had already shipped as 14.0.)
- API 16.1 covers both 3.27.5 (`MAXCHANNELS` 184) and master (230).

Two consequences:

1. **The generator must pin an exact commit, never an API pair** — already the
   plan's design (section 3), but now for a demonstrated reason rather than
   tidiness.
2. **`GPSDRaw14v0.msg` cannot be a straight 1:1 map for every libgps reporting
   14.0.** Generated from the pair's last rev it is a superset, and a parser
   naming `fix.jam` fails to compile against 3.24. Handled by D11.

Note also that the gps.h changelog is not reliable for values: it claims
`MAXCHANNELS` went to 185 and then 230, but no *release* ever shipped either —
140 and 184 are the only released values. Read the header, not the comment.

### 1.8 Existing code this builds on

The version-dispatch skeleton already exists and should be extended, not
replaced:

- [gpsd_client/include/gpsd_client/gpsd_parser.hpp](../gpsd_client/include/gpsd_client/gpsd_parser.hpp) — `GpsdParser` interface, `ParserContext`, the `API >= 9` `#error` guard
- [gpsd_client/include/gpsd_client/parsers/gpsd_parser_base.hpp](../gpsd_client/include/gpsd_client/parsers/gpsd_parser_base.hpp) — shared implementation
- [gpsd_client/src/gpsd_parser_factory.cpp](../gpsd_client/src/gpsd_parser_factory.cpp) — the single compile-time selection ladder
- [gpsd_client/src/client.cpp](../gpsd_client/src/client.cpp) — the component and its publishers
- [tools/test_against_gpsd.sh](../tools/test_against_gpsd.sh) — builds libgps from source per release and runs the parser tests against it
- [.github/workflows/gpsd_versions.yml](../.github/workflows/gpsd_versions.yml) — the matrix job that drives the above

Note the header-ordering hazard documented in `gpsd_parser.hpp:7-9`: `gps.h`
defines `STATUS_*` macros that collide with ROS message constants, so message
headers must be included *before* `gps.h`. Every new generated header must
respect this.

---

## 2. Architecture decisions

### D1 — `gps_msgs` never gains a libgps dependency

`gps_msgs` is a pure interface package released to the ROS build farm for five
distros. It must not learn about gpsd. **All** `GPSDRaw*` messages are generated
and built unconditionally, on every platform, whether or not libgps is present.
Version selection happens exclusively in `gpsd_client`.

Consequence: message content is pinned by the *generator run*, not by the build
host's gpsd. Regenerating is a deliberate, reviewed act.

### D2 — Messages are generated, not hand-written

Ten messages, each with hundreds of fields sourced from a C header that changes
between versions, is not maintainable by hand. Ground truth is
`include/gps.h` at a pinned commit per API pair.

- Generator: `tools/generate_raw_msgs.py`, checked in.
- Output: `.msg` files *and* the parser fill functions, both checked in — rosidl
  needs the `.msg` files in-tree, and reviewers need to see generated C++.
- CI verifies regeneration is a no-op (see phase 6), so the checked-in output
  cannot drift from the generator.

### D3 — Struct-per-message, version-suffixed

`gps_data_t` embeds structs, and `skyview[]` is an array of `satellite_t`, so
sub-messages are unavoidable. Those sub-structs also change shape between
versions, so they are versioned too:

```
GPSDRaw16v1.msg      -> gps_data_t
GPSDFix16v1.msg      -> gps_fix_t
GPSDSatellite16v1.msg-> satellite_t
GPSDDop16v1.msg      -> dop_t
...
```

Scalars declared directly in `gps_data_t` are flattened into the top-level
message. Only the ROS `std_msgs/Header` is added; every other field maps 1:1 to
a gpsd member, keeping the message honest to its `Raw` name.

### D4 — Fixed C arrays become unbounded ROS arrays

`skyview[MAXCHANNELS]` is sized 140 (gpsd 3.20-3.25) or 184 (3.26-3.27.5), with
230 so far only on master. Crucially the size is **not** a function of the API
pair: 14.0 covers both 140 and 184, and 16.1 covers both 184 and 230 (1.7).
Encoding it in the message type would therefore be not just wasteful but
ambiguous. Publish dynamic arrays truncated
to the valid count (`satellites_visible`, `devices.ndevices`, …).

### D5 — The union is represented by the `set` mask plus one populated arm

`gps_data_t` packs `rtcm2 / rtcm3 / subframe / ais / raw / osc / version / error`
into a union; which arm is live is indicated by `gps_data_t::set` against
`UNION_SET`. The message carries `uint64 set` verbatim plus a sub-message per
arm, and the parser populates **only** the arm the mask selects. Reading an
inactive arm from the union is UB; the parser must gate on the mask, not on
"looks nonzero".

The `ais` arm is excluded — see D10. Every other arm is represented.

### D6 — Scope tiers

Approximate member counts at API 16.1 (crude count from `gps.h`):

| Struct | ~members |
|---|---|
| `gps_fix_t` | 50 |
| `attitude_t` | 29 |
| `rawdata_t` | 17 |
| `satellite_t` | 14 |
| `devconfig_t` | 12 |
| `gps_policy_t` / `gst_t` | 11 each |
| `dop_t` | 7 |
| `oscillator_t` / `version_t` | 4 each |
| `subframe_t` | ~104 |
| `rtcm2_t` | ~93 |
| `rtcm3_t` | ~182 |
| ~~`ais_t`~~ | excluded per D10 |

"All information reported from GPSd" is the target — minus AIS (D10) — and the
tiers below are delivery order, not a further reduction in scope:

- **Tier A** — `fix`, `dop`, `skyview[]`, `satellites_used/visible`,
  `online`, `skyview_time`, `leap_seconds`, `set`, `status`. Covers everything a
  normal GNSS consumer wants.
- **Tier B** — `dev`, `devices[]`, `policy`, `gst`, `attitude`, `imu[10]`,
  `log`, `toff`, `pps`, `qErr`/`qErr_time`, `source`, `watch`.
- **Tier C** — union arms: `version`, `error`, `osc`, `raw`, `subframe`,
  `rtcm2`, `rtcm3`.

Non-goals, permanently excluded (not "information reported" — they are process-local):
`gps_fd`, `update_fd` (function pointer), `privdata`, `set_pending`.

### D10 — AIS is out of scope

The `ais` union arm (`struct ais_t`) is **not** represented in any `GPSDRaw*`
message. No `GPSDAis*` sub-message is generated and the parser never reads
`data.ais`.

`ais_t` is the largest and most awkward thing in `gps.h`: a tagged union keyed on
message `type` with roughly two dozen arms (types 1–27), several containing
further nested unions and variable-length payloads. It is also the least relevant
to this package — AIS is marine vessel traffic, decoded by gpsd as a convenience
because AIS receivers share NMEA plumbing; it is not GNSS data and has no bearing
on a fix. Consumers who want AIS are better served talking to gpsd's JSON
interface directly than through a versioned ROS mirror of a C union.

Consequences to implement deliberately:

- **`SET_AIS` is still emitted.** The mask constant is part of the raw mask, and
  `set` is carried verbatim per D5. A consumer can therefore detect exactly the
  case "gpsd reported AIS data here and this message does not carry it" by
  testing `set & SET_AIS`. That is honest; silently clearing the bit would not be.
- **`SET_UNION` keeps `AIS_SET` in its composite**, matching gpsd's own
  `UNION_SET` definition. Do not redefine it to exclude AIS — the constant must
  mean what `gps.h` says it means.
- **When `set & AIS_SET` is live, no union arm is populated.** This is the one
  case where the mask selects an arm the message has no field for. The parser
  leaves every arm default-constructed and returns normally — it must not fall
  through to reading a different arm, which would be UB.
- The generator needs no `ais_t` support at all, which removes the nested-union
  parsing work from phase 1.

### D7 — Publishing is opt-in and type-erased at the edge

New parameter `publish_gpsd_raw` (bool, default `false`), new topic `gpsd_raw`.
Because exactly one raw message type is live per build, `gpsd_client` exposes a
single alias resolved by the same `#if` ladder as the parser:

```cpp
// gpsd_raw_message.hpp
#if   GPSD_API_MAJOR_VERSION == 9 && GPSD_API_MINOR_VERSION == 0
using GpsdRawMsg = gps_msgs::msg::GPSDRaw9v0;
#elif ...
```

`client.cpp` then names `GpsdRawMsg` once and needs no further version logic.
Default `false` so existing deployments see no new traffic or CPU cost; a full
`gps_data_t` at 230 skyview entries is not free to serialize at 10 Hz.

### D8 — Parser naming shifts from "highest supported" to exact pair

The current convention names a parser after the highest API it supports
(`GpsdParserV16` covers 10–16). Raw messages are exact-per-pair, so raw fill
logic is named exactly: `GpsdRawParser9v0`, `GpsdRawParser16v1`, … The existing
`GpsdParserV9` / `GpsdParserV16` split for `GPSFix`/`NavSatFix` stays as-is —
this work adds a third output, it does not restructure the first two.

### D9 — Emit the `set` bitmask constants, renamed `SET_<NAME>` to dodge `gps.h`

**Decided: yes, emit them.** Each `GPSDRaw<M>v<m>.msg` carries the mask bits
valid for its API version as `uint64` constants, so a consumer can test
`data.set & Msg::SET_LATLON` without linking libgps. Per 1.5 the bit positions
are append-only, so these values are stable across versions.

**But they cannot keep gpsd's spelling.** rosidl emits message constants as
`static constexpr` members of the message struct — confirmed in the existing
generated header:

```cpp
// gps_msgs/msg/detail/gps_status__struct.hpp:188
static constexpr int16_t STATUS_NO_FIX = ...
```

`gps.h` defines every mask bit as a *global preprocessor macro*
(`#define STATUS_SET (1llu<<9)`, `LATLON_SET`, …). A member declaration named
`STATUS_SET` in a header parsed after `gps.h` is textually destroyed by the
preprocessor, producing an incomprehensible error.

This is not hypothetical — the codebase is already paying for exactly this
collision. [gpsd_parser_base.cpp:41-49](../gpsd_client/src/parsers/gpsd_parser_base.cpp)
documents it, and the mapping helpers below that comment hardcode raw integers
(`return 18;  // gps_msgs::msg::GPSStatus::STATUS_DGPS_FIX`) because the named
constants are unusable there. The include-order rule in
[gpsd_parser.hpp:7-9](../gpsd_client/include/gpsd_client/gpsd_parser.hpp)
is the current workaround.

Keeping gpsd's names would extend that hazard from a handful of `STATUS_*`
identifiers to ~45, and — worse — push it onto downstream users. `gps_msgs` is
libgps-free by D1 and gets consumed by nodes we don't control; one that writes

```cpp
#include <gps.h>                            // any order it likes
#include <gps_msgs/msg/gpsd_raw16v1.hpp>    // now broken
```

has no way to know the include order is load-bearing.

**Naming rule:** gpsd `<NAME>_SET` → message `SET_<NAME>`. Mechanical, reversible,
and collision-free because `gps.h` defines no `SET_*` macros. Examples:

| gpsd macro | message constant |
|---|---|
| `ONLINE_SET` | `SET_ONLINE` |
| `LATLON_SET` | `SET_LATLON` |
| `STATUS_SET` | `SET_STATUS` |
| `SPARTN_SET` (API 16+) | `SET_SPARTN` |

Values are emitted verbatim; only the identifier changes. Also emit
`SET_UNION` (gpsd's `UNION_SET` composite) since D5 gating needs it, and
`SET_HIGH_BIT` as a plain `uint64` marking the highest defined bit for that
version. The `uint64 set` field itself stays raw and undecoded.

### D11 — Absent members are detected in C++, not in CMake

Because an API pair spans a range of header states (1.7), a message generated
from the pair's last rev can name `gps_fix_t` members that an older libgps
reporting the *same* pair does not have. `fix.jam` compiles against gpsd 3.26.1
and fails against 3.24, both API 14.0.

Per-field `check_cxx_symbol_exists` does not scale here — it is one CMake probe
and one `-D` per divergent field, and `check_cxx_symbol_exists` cannot see a
struct member anyway. Use a C++17 detection idiom instead, which the generator
emits automatically since it already knows every field name:

```cpp
#define GPSD_DEFINE_HAS_MEMBER(name)                                       \
  template <typename T, typename = void>                                   \
  struct has_##name : std::false_type {};                                  \
  template <typename T>                                                    \
  struct has_##name<T, std::void_t<decltype(std::declval<T&>().name)>>     \
      : std::true_type {};
```

Each generated field assignment is then guarded by
`if constexpr (has_<name><gps_fix_t>::value)`, leaving the message field at its
default when the build's libgps lacks the member.

Verified against three real installs — the discrimination is exactly where the
API version cannot reach:

| Build | `latitude` | `jam` | `clockbias` | `wtemp` |
|---|---|---|---|---|
| 3.20 (API 9.0) | yes | no | no | no |
| 3.24 (API **14.0**) | yes | **no** | **no** | **no** |
| 3.27.5 (API 16.1) | yes | yes | yes | yes |

The header-audit test (5.4 #2) remains the backstop: it reads the *build's*
gps.h, so it still fails loudly if a member exists and nothing maps it.

---

## 3. Message inventory

Ten messages. `[ ]` per message tracks Tier A/B/C completion.

| Message | API | Reference gpsd rev | Tier A | Tier B | Tier C |
|---|---|---|---|---|---|
| `GPSDRaw9v0.msg` | 9.0 | `release-3.20` | [ ] | [ ] | [ ] |
| `GPSDRaw9v1.msg` | 9.1 | `e5279ef52` (2020-03-19) | [ ] | [ ] | [ ] |
| `GPSDRaw10v0.msg` | 10.0 | `release-3.21` | [ ] | [ ] | [ ] |
| `GPSDRaw10v1.msg` | 10.1 | `42f816d59` (2020-08-21) | [ ] | [ ] | [ ] |
| `GPSDRaw11v0.msg` | 11.0 | `release-3.22` | [ ] | [ ] | [ ] |
| `GPSDRaw12v0.msg` | 12.0 | `release-3.23.1` | [ ] | [ ] | [ ] |
| `GPSDRaw13v0.msg` | 13.0 | `264e808c6` (2022-04-06) | [ ] | [ ] | [ ] |
| `GPSDRaw14v0.msg` | 14.0 | `release-3.26.1` | [ ] | [ ] | [ ] |
| `GPSDRaw16v0.msg` | 16.0 | `release-3.27.3` | [ ] | [ ] | [ ] |
| `GPSDRaw16v1.msg` | 16.1 | `release-3.27.5` | [ ] | [ ] | [ ] |

Every message starts with, following the `GPSFix.msg` convention:

```
# Raw gpsd report (gps_data_t) as delivered by libgps API <MAJOR>.<MINOR>.
# Generated by tools/generate_raw_msgs.py from gps.h at <rev>. Do not edit.
std_msgs/Header header
```

---

## 4. Type mapping rules

Fixed at the start of phase 1, applied by the generator, recorded here so
reviewers can check the generated output against a stated rule.

| gpsd C type | ROS 2 field | Note |
|---|---|---|
| `double` | `float64` | |
| `float` | `float32` | |
| `int` / `unsigned` | `int32` / `uint32` | |
| `short` / `unsigned short` | `int16` / `uint16` | |
| `long` / `unsigned long` | `int64` / `uint64` | Width differs by ABI; widen to 64 |
| `bool` | `bool` | |
| `char name[N]` | `string` | NUL-terminated, truncate at first NUL |
| `char data[N]` (binary) | `uint8[]` | Not a string — see `rtcm3` payloads |
| `timespec_t` | `builtin_interfaces/Time` | `tv_sec`/`tv_nsec` map directly |
| `time_t` | `int64` | Seconds; not a ROS `Time` (may be a duration/TOW) |
| `gps_mask_t` | `uint64` | Verbatim; do not decode |
| `gnssid_t` (API 16+) | `uint8` | Was plain `unsigned char` before |
| `struct X` | `GPSD<X><MAJOR>v<MINOR>` | Per D3 |
| `struct X arr[N]` | `GPSD<X><MAJOR>v<MINOR>[]` | Unbounded, per D4 |
| C union | mask + all arms, one populated | Per D5 |
| NaN sentinel | `float64` NaN preserved | gpsd uses NaN for "unknown"; do not zero |

The `set` bitmask constants are emitted per D9, renamed `<NAME>_SET` →
`SET_<NAME>`. The generator derives them from the `#define <NAME>_SET (1llu<<N)`
lines in the target rev's `gps.h`, so each message defines exactly the bits its
API version knows about.

---

## 5. Test data strategy

Yes — gpsd ships two independent ways to produce synthetic reports, and they
have complementary blind spots. Use both.

### 5.1 Tier 1: `gps_unpack()` — JSON string straight into `gps_data_t`

`gps.h` publicly declares:

```c
extern int gps_unpack(const char *, struct gps_data_t *);
```

Confirmed present in **every** version in our range (3.20 → 3.27.5) and exported
from `libgps.so` (`nm -D` shows `T gps_unpack`). Critically, this is not a
side door: `gps_read()` calls it on the socket buffer at
`libgps/libgps_sock.c:375`, so a test that feeds it a JSON string exercises the
exact code path a live client takes, minus the socket.

This is the workhorse for field coverage:

```cpp
gps_data_t data{};
gps_unpack(R"({"class":"TPV","mode":3,"lat":42.0,"lon":-83.0,...})", &data);
auto msg = parser->parseRaw(data, stamp);
EXPECT_DOUBLE_EQ(msg.fix.latitude, 42.0);
```

No daemon, no device, no pty, no network, no timing. Fast enough to run one case
per field.

### 5.2 Tier 2: `gpsfake` — real daemon, real device logs

`gpsfake` (`gpsfake.py.in`, logic in `gps/fake.py`) replays a recorded device log
into a **real `gpsd` daemon** over a synthetic device, then lets real clients
connect. Our node connects via `gpsmm` exactly as in production, so this is the
only tier that tests the whole chain end to end.

Key API (`gps/fake.py`):

| Piece | Role |
|---|---|
| `TestSession(prefix, port, options, verbose)` | Spawns and manages the `gpsd` daemon; `port` is what our node's `port` parameter points at |
| `TestSession.gps_add(logfile, speed, pred, oneshot)` | Attaches a fake GPS replaying `logfile` |
| `TestSession.client_add(commands)` | Attaches a client |
| `FakePTY` / `FakeTCP` / `FakeUDP` | Three transports; **prefer `FakeTCP`/`FakeUDP` in containers**, which avoid pty allocation entirely |
| `TestSession.wait()` / `.cleanup()` | Lifecycle |

The corpus is `test/daemon/` in the gpsd tree: **196 recorded device logs**, each
with a `.log.chk` file holding the JSON gpsd is expected to emit for it. The
`.chk` files double as ground truth — if our message disagrees with the `.chk`,
one of the two is wrong.

Representative logs per report class, counted across the corpus:

| Class | Logs | Representative | Maps to |
|---|---|---|---|
| `TPV` | 177 | `ac12_binary` | Tier A `fix` |
| `SKY` | 164 | `ac12` | Tier A `skyview[]`, `dop` |
| `GST` | 13 | `gr8013-w` | Tier B `gst` |
| `ATT` | 11 | `hemi` | Tier B `attitude` |
| `RTCM3` | 7 | `ublox-zed-f9r` | Tier C `rtcm3` |
| `RTCM2` | 4 | `naujoks-rtcm2` | Tier C `rtcm2` |
| `SUBFRAME` | 4 | `skytraq-bin` | Tier C `subframe` |
| `RAW` | 3 | `ublox-neo-m8t` | Tier C `raw` |
| `OSC` | 3 | `ericsson-gru04` | Tier C `osc` |
| `IMU` | 2 | `ublox-neo-m8u` | Tier B `imu[]` |
| `LOG` | 2 | `ublox-zoe-m8b-logbatch` | Tier B `log` |
| `AIS` | — | (excluded per D10) | — |

**Two gaps gpsfake cannot close**, which is exactly why tier 1 exists:

- `TOFF` and `PPS` appear in **zero** logs. They need real PPS hardware; log
  replay cannot produce them. `toff`/`pps`/`qErr` are reachable only via
  `gps_unpack()`.
- `VERSION`, `DEVICE` and `WATCH`/policy appear in no `.chk` file because the
  *daemon* emits them on connect rather than the device. They do arrive in a live
  gpsfake session, but they are not in the corpus to assert against.

### 5.3 Cost of adopting gpsfake

Not free, and this is a real change to the build:

- [tools/test_against_gpsd.sh:132](../tools/test_against_gpsd.sh) currently builds
  **libgps only** — `gpsd=False gpsdclients=False python=False`. gpsfake needs
  both the `gpsd` daemon binary (`gps/fake.py:640` spawns it) and the `gps`
  Python module. Both flags must flip for gpsfake tests, which lengthens each of
  the ten matrix builds.
- `gpsfake` hard-asserts `gps.__version__ == gps_version` and exits if they
  differ, so the Python module must come from the *same* build as the daemon.
  A system-packaged gpsfake cannot be mixed with a source-built gpsd.

**Recommendation:** tier 1 runs in every CI job across all ten API versions —
it is fast and needs only libgps. Tier 2 runs as a **separate job on the newest
version only**, where it validates that the whole chain works; the per-version
correctness argument is carried by tier 1. Revisit if a version-specific
end-to-end bug ever slips through.

### 5.4 How "all data is published" is actually guaranteed

Coverage claims by hand-written test are worth little on a struct this size.
Two mechanical checks, both generator-emitted:

1. **Round-trip coverage test (per API pair, generated).** The generator already
   knows every field it mapped. It emits a test that writes a *distinct sentinel*
   into each `gps_data_t` member, runs the parser, and asserts each ROS field
   equals its sentinel. Distinct values catch copy-paste field mix-ups that a
   uniform fill would hide. Because it is generated from the same field model as
   the parser, it cannot drift out of sync.

2. **Header audit test (the real completeness guarantee).** Test 1 only proves
   the fields the generator *knew about* survive; it says nothing about a field
   the generator silently skipped. So: enumerate every member of `gps_data_t`
   and its nested structs from that version's `gps.h`, and assert each is either
   (a) mapped to a message field, or (b) on an explicit exclusion list — `ais`
   (D10), `gps_fd`, `update_fd`, `privdata`, `set_pending` (D6). A new gpsd
   release adding a field then **fails the build** with a named, actionable
   error instead of silently dropping data.

Exclusions live in one list in the generator, so "what we deliberately do not
publish" is a single reviewable place rather than an emergent property.

---

## 6. Implementation phases

### Phase 0 — Groundwork  *(complete)*

- [x] Confirm this plan's API↔release table against a fresh `git fetch --tags` of upstream gpsd — fetched, no new tags, 3.27.5 still latest; all 14 releases ≥ 3.20 re-read and the table holds
- [x] Confirm `MAXCHANNELS` per reference rev — **expectation was wrong.** Released values are 140 (3.20-3.25) and 184 (3.26-3.27.5) only; 185 never existed and 230 is master-only. Corrected in 1.7 and D4
- [x] Decide the exact reference commit for the three unreleased pairs — resolved as `<next bump>^` and each verified to report the intended pair: 9.1 → `e5279ef52`, 10.1 → `42f816d59`, 13.0 → `264e808c6` (all `MAXCHANNELS` 140)
- [x] Record the type-mapping table (section 4) as the generator's docstring — [tools/generate_raw_msgs.py](../tools/generate_raw_msgs.py), which also carries the pinned reference-rev manifest
- [x] *(unplanned)* Established that API pairs span ranges of header states, not single states (1.7), and validated the C++ member-detection idiom that handles it (D11)

### Phase 1 — Generator  *(complete for Tier A)*

- [x] `tools/generate_raw_msgs.py`: parse `gps.h` at a given rev → structured field model
  - [x] Handle preprocessor conditionals — only `#ifndef USE_QT` (around the already-excluded `gps_fd`) exists in the whole range; anything else raises rather than emitting both arms. Line-continued `#define`s (`UNION_SET`) are joined before parsing
  - [x] Handle anonymous structs/unions — inline `struct { … } ecef;` becomes its own message; a *declarator-less* anonymous union has its members spliced into the parent per C11 6.7.2.1, which is what makes the tier filters and the AIS exclusion match by plain name
  - [ ] Handle nested unions inside `subframe_t` (Tier C; `ais_t` skipped per D10)
- [x] Parse the mask block into `SET_<NAME>` constants (D9), including the composite `UNION_SET` → `SET_UNION` (value cross-checked independently) and `SET_HIGH_BIT`
- [x] Emit `.msg` files into `gps_msgs/msg/` — 70 messages (7 per pair x 10 pairs)
- [x] Emit parser fill code — as headers under `gpsd_client/include/gpsd_client/parsers/generated/`, not `src/`, since the fill functions are templates (see below)
- [x] `--check` mode — verified both directions: passes clean, exits 1 on an injected one-line drift
- [x] Unit tests — [tools/test_generate_raw_msgs.py](../tools/test_generate_raw_msgs.py), 23 tests over fragments, no gpsd checkout needed
- [x] Output tests — [tools/test_generated_messages.py](../tools/test_generated_messages.py), 30 tests asserting the generated messages match the real `gps.h` per API version (see below)
- [x] All tests run under standard ROS tooling: plain `colcon test` reports **77 tests, 0 failures, 0 skipped** against gpsd 3.20, 3.24 and 3.27.5

Verified end to end, not just generated:

- `gps_msgs` builds all 70 generated messages (57s, no libgps present — D1 holds)
- Generated `#include` paths match rosidl's real header names exactly, including the acronym split (`gpsd_baseline16v1`, not `gpsdbaseline16v1`) — the first attempt got this wrong and was corrected against the built output
- The generated `fill()` for 16.1 compiles against real libgps 3.27.5 and produces correct values (lat/lon/leap/hdop/timespec to `builtin_interfaces/Time`)
- **D11 confirmed under the exact condition it exists for:** `gpsd_raw_fill_14v0.hpp` (generated from 3.26.1) compiles against gpsd **3.24**, filling what exists and leaving `jam`/`temp`/`clockbias` at defaults; on a 3.27.5 build the `#if` pair guard compiles it out entirely
- `gpsd_client` still builds and both test suites still pass against 3.20, 3.24 and 3.27.5

Message correctness is checked two independent ways, because the fragment
tests alone would pass while the generator emitted a message that had nothing
to do with the gpsd version it names:

- **Manual expectations** — hand-written per-version tables tied to specific
  gpsd changes: `status` living in `gps_data_t` only on API 9, `leap_seconds`
  arriving at 9.1, `baseline_t` at 13.0, the six mid-pair API-14 `gps_fix_t`
  additions, the "API 15" members surfacing as 16. A failure names the gpsd
  change it broke.
- **Independent cross-checks** — struct members and their C types are
  re-extracted from the real `gps.h` by a deliberately *separate* scanner, and
  every non-excluded member must have a correctly typed field (and vice versa:
  no field without a member). Reusing the generator's own parser here would be
  circular. Both directions carry a guard-on-the-guard assertion so a scanner
  that silently returned nothing cannot make the suite pass vacuously.

The suite was **mutation-tested**: twelve deliberate generator bugs (dropped
member, wrong reference rev, gpsd `_SET` spelling, removed AIS exclusion,
`double`→`float32`, `int16_t`→`int32`, `timespec_t` filled as a scalar, missing
version suffix, `char[N]`→`uint8[]`, dropped `if constexpr` guard, blind
`skyview` fill, wrong API-pair `#if`) — all twelve are caught. The first pass
caught only 8; the type-mapping and generated-C++ mutations survived and are
why `FieldTypes` and `GeneratedParserCode` exist.

Two implementation notes worth carrying forward:

- Fill functions are **templated on the source type**. The anonymous structs (`gps_fix_t::ecef`, `::NED`) have no C type name to write down, and deducing `T` is what lets the `has_<member><T>` traits resolve against the build's real `gps.h`. They need forward declarations, since unqualified lookup in a template happens at definition time and ADL cannot reach `gpsd_client::generated`.
- Arrays of structs (`skyview[]`) are deliberately *not* filled by generated code — only the hand-written parser knows the valid count, and a blind loop would publish `MAXCHANNELS` entries of garbage. Phase 3 wires that up.

### Phase 2 — Messages (`gps_msgs`)

- [ ] Generate Tier A for all ten messages
- [ ] Add generated `.msg` files to `MSG_FILES` in [gps_msgs/CMakeLists.txt](../gps_msgs/CMakeLists.txt)
- [ ] Add `builtin_interfaces` to `MSG_DEPS` and `package.xml` (needed for `timespec_t`)
- [ ] Confirm `gps_msgs` builds standalone with no libgps present (D1)
- [ ] Assert no generated constant name collides with a `gps.h` macro — grep the generated headers for `\b[A-Z0-9_]+_SET\b` and fail if any hit (D9 regression guard)
- [ ] Compile-test the reverse include order (`gps.h` first, then a `GPSDRaw*` header) in a throwaway TU; it must build, proving D9 removed the ordering dependency for downstream users
- [ ] Extend to Tier B
- [ ] Extend to Tier C
- [ ] Decide whether `ros1_ros2_mapping.yaml` needs entries (probably not — no ROS 1 counterpart exists)

### Phase 3 — Parsers (`gpsd_client`)

- [ ] `gpsd_client/include/gpsd_client/gpsd_raw_message.hpp` — the `GpsdRawMsg` alias ladder (D7)
- [ ] `GpsdRawParser` interface: `parseRaw(const gps_data_t&, const rclcpp::Time&) -> GpsdRawMsg`
- [ ] Per-pair generated implementations `gpsd_raw_parser_<M>v<m>.{hpp,cpp}`, each guarded on its exact `(MAJOR, MINOR)`
- [ ] Extend `GpsdParserFactory` with `createRaw()`; keep it as the *only* selection ladder (the existing file comment says so — honor it)
- [ ] `#error` for API < 9; `#warning` + newest parser for API > 16, matching the existing fallback policy in `gpsd_parser_factory.cpp:17-23`
- [ ] Verify header include order: message headers before `gps.h` everywhere (`gpsd_parser.hpp:7-9`)

### Phase 4 — Publishing (`client.cpp`)

- [ ] Declare `publish_gpsd_raw` parameter (default `false`)
- [ ] Create the `gpsd_raw` publisher only when enabled
- [ ] Populate and publish in `step()` alongside the existing `GPSFix` / `NavSatFix` publishes
- [ ] Add `publish_gpsd_raw` to [gpsd_client/config/gpsd_client.yaml](../gpsd_client/config/gpsd_client.yaml)
- [ ] Confirm the built-in default matches the config file — the README explicitly promises these agree

### Phase 5 — Tests

See section 5 for the data-generation strategy behind these.

**Tier 1 — `gps_unpack()`, all ten API versions:**  *(harness landed)*

- [x] Harness: `test/gpsd_json_fixture.{hpp,cpp}` — `makeEmptyData()` (mirrors `gps_open()`), `unpack()`, and `tpvJson()`/`skyJson()` builders
- [x] CMake probes for `gps_clear_gst`/`gps_clear_log` (see 1.7); new `test_gpsd_json_fixture` target
- [x] Verified building **and running** against API 9.0 (3.20), 14.0 (3.24) and 16.1 (3.27.5): 10/10 pass on each, and the pre-existing `test_gpsd_parser` still passes on all three
- [ ] Extend [gpsd_client/test/test_gpsd_parser.cpp](../gpsd_client/test/test_gpsd_parser.cpp) with raw-parser cases
- [ ] Helper: JSON string → `gps_data_t` via `gps_unpack()`, asserting its return status
- [ ] Generated round-trip coverage test per API pair: distinct sentinel per field, assert each arrives (5.4 #1)
- [ ] Generated header-audit test: every `gps_data_t` member is mapped or explicitly excluded (5.4 #2) — this is what makes a new gpsd field a build failure rather than silent data loss
- [ ] `TOFF`/`PPS`/`qErr` cases — reachable only here, never via gpsfake (5.2)
- [ ] Round-trip test: populate a synthetic `gps_data_t` → parse → assert every field
- [ ] Union test: set each `UNION_SET` bit in turn, assert only that arm is populated (D5)
- [ ] AIS test: with `AIS_SET` live, assert the parser returns normally, populates no union arm, and leaves `SET_AIS` visible in the message's `set` field (D10)
- [ ] Mask-constant test: `static_assert` each `SET_<NAME>` equals `gps.h`'s `<NAME>_SET` for the API being built against (D9) — this is the one place both spellings are legitimately in scope, so it is the natural place to catch a generator mistake
- [ ] NaN preservation test (gpsd's "unknown" sentinel must survive)
- [ ] String truncation test (`char[N]` without a NUL)
- [ ] Test must compile under every API in the matrix — guard version-specific assertions

**Tier 2 — gpsfake end-to-end, newest version only:**

- [ ] Rebuild that version's gpsd with `gpsd=True python=True` (see 5.3 for the cost)
- [ ] Launch-test: `gps.fake.TestSession` on a free port → `gpsd_client` node with `host`/`port` pointed at it → capture published topics
- [ ] Use `FakeTCP`/`FakeUDP` rather than `FakePTY` to avoid pty allocation in containers
- [ ] Assert `GPSFix`, `NavSatFix` and `gpsd_raw` all publish, with `publish_gpsd_raw` enabled
- [ ] Cross-check published values against the log's `.log.chk` ground truth
- [ ] Cover one log per report class from the 5.2 table (`ac12`, `hemi`, `gr8013-w`, `ublox-zed-f9r`, `skytraq-bin`, `ublox-neo-m8t`, `ericsson-gru04`, `ublox-neo-m8u`, `ublox-zoe-m8b-logbatch`)
- [ ] Confirm `gps.__version__` matches the built daemon, or the test aborts unhelpfully (5.3)

### Phase 6 — CI

- [ ] Teach `tools/test_against_gpsd.sh` to accept a commit SHA as well as a release tag (needed for 9.1, 10.1, 13.0 — `build_gpsd()` currently hardcodes `git checkout release-${ver}`)
- [ ] Read `GPSD_API_MINOR_VERSION` too — `api_version()` currently reads only the major
- [ ] Update `parser_for_api()` to also report the selected *raw* parser
- [ ] Expand the matrix in [.github/workflows/gpsd_versions.yml](../.github/workflows/gpsd_versions.yml) from `['3.20','3.21','3.27.5']` to one entry per API pair:

  | Matrix entry | API pair |
  |---|---|
  | `3.20` | 9.0 |
  | `8da63ed3` | 9.1 |
  | `3.21` | 10.0 |
  | `7c7de250` | 10.1 |
  | `3.22` | 11.0 |
  | `3.23.1` | 12.0 |
  | `81115741` | 13.0 |
  | `3.26.1` | 14.0 |
  | `3.27.3` | 16.0 |
  | `3.27.5` | 16.1 |

- [ ] Add a `generate --check` job so checked-in generated files cannot drift (D2)
- [ ] Verify the cache key still discriminates (`gpsd-${{ runner.os }}-${{ matrix.gpsd }}-v2` works for SHAs, but bump to `-v3` when the matrix changes shape)
- [ ] Watch job count/time: 10 source builds of gpsd. If wall-clock becomes a problem, keep the full matrix on `ros2-devel` pushes (the workflow is already gated that way at `gpsd_versions.yml:15`) and run a 3-entry subset on PRs

### Phase 7 — Docs

- [ ] README: `publish_gpsd_raw` row in the parameter table, and a `gpsd_raw` section
- [ ] Document that API 15 does not exist and why there is no `GPSDRaw15v0` (section 1.2)
- [ ] Document that AIS is not carried, and that `set & SET_AIS` is how a consumer detects an AIS report the message omits (D10)
- [ ] Document the `SET_<NAME>` constants and why they are not spelled `<NAME>_SET` (D9) — users coming from the gpsd docs will look for the gpsd spelling
- [ ] `gps_msgs/CHANGELOG.rst` and `gpsd_client/CHANGELOG.rst`
- [ ] Document the "how to add the next API version" procedure — this will happen again

---

## 7. Open questions

Not blocking; recorded so the choice is deliberate when each is reached.

1. **Unreleased pairs.** 9.1, 10.1, 13.0 shipped in no gpsd release. Generating
   messages for them costs little; testing them costs three extra CI builds
   against commit SHAs. Plan assumes include-and-test. Drop them if CI time
   becomes the binding constraint — say so here if so.
2. **Tier C size.** With AIS excluded (D10), `rtcm3_t` (~182 members),
   `subframe_t` (~104) and `rtcm2_t` (~93) are what remains of the bulk, and are
   still the least-used data. If Tier C lands late or partially, that should be
   an explicit, documented decision rather than a silent gap.
3. **Serialization cost at rate.** Measure Tier C at 10 Hz before declaring the
   feature done. If it's heavy, consider whether the union arms warrant a
   separate topic — but only with measurements in hand.

---

## 8. Progress log

Newest first. One line per meaningful step: what changed, and anything the next
person needs to know that isn't obvious from the diff.

| Date | Phase | Note |
|---|---|---|
| 2026-08-16 | 1/5 | Generated-output tests added (30) and all Python tests wired into `colcon test` via `gps_msgs` + `ament_add_pytest_test`; drift check now runs as an ordinary test. Fixed `colcon test` failing to find a source-built libgps by adding `APPEND_LIBRARY_DIRS` to both gtest targets. Whole workspace: 77 tests, 0 failures, 0 skipped on gpsd 3.20/3.24/3.27.5. Mutation-tested the suite: 12/12 injected generator bugs caught. |
| 2026-08-16 | 1 | Generator implemented for Tier A: 70 messages + 10 parser headers + `gpsd_has_member.hpp`, 23 unit tests, `--check` verified both ways. `gps_msgs` builds all of them; generated fill code verified running against real libgps; D11 confirmed against gpsd 3.24. Corrected the emitted `#include` paths after checking them against rosidl's actual output. |
| 2026-08-16 | 0 | Phase 0 complete. Upstream refetched (no new tags). Two corrections: `MAXCHANNELS` released values are 140/184, not 140/185/230; and API pairs span *ranges* of header states, not single states — `gps_fix_t` grows by six members within API 14.0, and 16.1 covers both 184 and 230. Added D11 (C++ member detection, validated on three installs) to handle it. Reference revs pinned and all ten verified in [tools/generate_raw_msgs.py](../tools/generate_raw_msgs.py). |
| 2026-08-16 | 5 | Tier-1 harness landed and verified on API 9.0/14.0/16.1. Found three libgps changes that do *not* track the API version (new section 1.7) — `gps_unpack()`'s constness, `gps_clear_gst()`'s existence, and the SKY `nSat` requirement. Also confirmed no released gpsd propagates parse errors out of `gps_unpack()`, so fixture typos surface as value assertions, never as a parse failure. |
| 2026-08-16 | 5 | Test data strategy added (new section 5; later sections renumbered). Two tiers: `gps_unpack()` for per-field coverage on all ten versions, gpsfake for end-to-end on the newest. Completeness is enforced by a generated header-audit test, not by hand-written cases. |
| 2026-08-16 | — | D10: AIS excluded. No `ais` union arm, no `GPSDAis*` sub-message, no `ais_t` support in the generator. `SET_AIS` is still emitted and `set` still carries the bit, so an omitted AIS report stays detectable. |
| 2026-08-16 | — | gpsd 3.27/3.27.1/3.27.2 (`api_version_major == 0`) declared out of scope. No coverage lost — all three are C API 16.0, already represented by `release-3.27.3`. Dropped the phase 7 task to document the quirk. |
| 2026-08-16 | — | D9 decided: emit the `set` bitmask constants. Renamed `<NAME>_SET` → `SET_<NAME>` because rosidl emits constants as `static constexpr` members and `gps.h` defines the gpsd spellings as global macros. Open question 3 closed. |
| 2026-08-16 | — | Plan created. API↔release table derived from gpsd git history; API 15 confirmed never to have existed. |
