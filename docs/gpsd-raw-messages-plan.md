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
| SKY `nSat` key required | absent (2021-11-23) | required (2022-04-06) | 13.0 **both sides** |
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
- The SKY `nSat` key landed **mid-API-13**, one day before the bump to 14
  (absent at `81115741`, present at `264e808c6`, bump at `0df348fe`). A
  threshold keyed on "API >= 14" is therefore wrong for the second half of API
  13 — which is exactly the reference revision this plan pins for `GPSDRaw13v0`.

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

**Pointer members are excluded as a class**, by type rather than by name. The
motivating case is `fixsource_t`, whose `server`/`server_ip`/`port`/`device`
are `const char *`: `gps_open()` stores the caller's host and port pointers
verbatim and never copies them (`libgps/libgps_core.c`). `fixsource_t::spec` is
a real `char[512]` carrying the same information and *is* published, so nothing
is lost.

That exclusion also surfaced a live defect in this package — see D12.

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

### D12 — `gpsd_client` must own the host and port strings

Found while adding Tier B. `client.cpp` built the gpsmm connection from
*locals*:

```cpp
std::string host = "localhost";     // dies when start() returns
char port_s[12];
gps_ = std::make_unique<gpsmm>(host.c_str(), port_s);
```

`gpsmm` forwards to `gps_open()`, which stores both pointers in
`gps_data_t::source` and never copies them. Once `start()` returned, gpsd's own
record of where its data came from pointed at freed stack memory, for the whole
life of the node.

It was dormant: libgps never reads those fields back, and this project
deliberately does not publish them (D6). But gpsd's own clients — `cgps`,
`gpspipe`, `gps2udp` — all read `source.server`/`source.port`, so it is a
perfectly ordinary thing to reach for, and `source` is part of every report
handed to the parsers.

Confirmed rather than assumed, with AddressSanitizer against real libgps and no
daemon (the pointers are stored before the connection is attempted):

```
OLD (locals):  ERROR: AddressSanitizer: stack-use-after-return
               READ of size 10 ...
NEW (members): source.server = localhost
               source.port   = 2947
```

**Fix:** `host_` and `port_` are members, declared *before* `gps_` so that
reverse-order destruction leaves them alive longer than the gpsmm holding
pointers into them, and never reassigned after the connection is built.

### D13 — Generated messages live in `gps_extended_msgs`

Tier C makes the generated set ~905 messages, 90 per API pair, and building
them takes **8m48s** (measured) against 1m48s for Tier A+B alone and ~8s for
`gps_msgs` by itself.

`gps_msgs` is a small, long-released interface package — `GPSFix`, `GPSStatus`
— that the ROS build farm builds for five distros and that many downstreams
depend on. Putting the generated set there would impose a nine-minute build on
every one of them, released or not, whether or not they want raw gpsd data.

**Decision:** the generated messages live in a new `gps_extended_msgs` package.
`gps_msgs` returns to its prior contents and build time; `gpsd_client` depends
on both.

The message names keep the `GPSD` prefix — they mirror gpsd's own structures,
and the package name already says how they relate to `gps_msgs`. (They were
briefly `GPSExtended*`; the rename back is what exercised D15's orphan
removal, which deleted all 905 stale files on the next run.)

This supersedes the original placement in `gps_msgs/msg`. D1 is unchanged in
substance — the message package still never gains a libgps dependency — it just
applies to `gps_extended_msgs` now.

### D14 — Message names must already be in rosidl's normalised form

`camel()` lower-cases the tail of an acronym: `gps_fix_t::NED` becomes `Ned`,
not `NED`. That looks like a loss of fidelity and was "fixed" once; the fix
broke the build.

rosidl normalises a run of capitals when deriving the C struct name
(`GPSDFixNED16v1` → `gps_extended_msgs__msg__GPSDFixNed16v1`) but
writes the name *as authored* into the referencing message's header. The two
spellings disagree and the generated C fails with `unknown type name ...NED16v1;
did you mean ...Ned16v1?`.

Guarded by a test asserting no generated message name contains consecutive
capitals after the fixed prefix.

### D15 — The generator removes what it no longer produces

Writing files without removing stale ones is not merely untidy here: the
message package **globs** its directory, so a file left behind by a rename is
still built. That is exactly how the D14 rename briefly produced two
conflicting definitions of the same message and broke the build even after the
generator was corrected.

`--check` now reports orphans as drift, and a normal run deletes them. The scan
is scoped to the directories the generator owns and to its own naming, so it
can never propose deleting a hand-written file.

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

### Phase 2 — Messages (`gps_msgs`)  *(Tier A + B complete)*

- [x] Generate Tier A for all ten messages
- [x] Add generated `.msg` files to `MSG_FILES` in [gps_msgs/CMakeLists.txt](../gps_msgs/CMakeLists.txt) — globbed, so regenerating needs no CMake edit
- [x] Add `builtin_interfaces` to `MSG_DEPS` and `package.xml` (needed for `timespec_t`)
- [x] Confirm `gps_msgs` builds standalone with no libgps present (D1)
- [x] Assert no generated constant collides with a `gps.h` macro — done in the generator against the full macro namespace, not a name pattern (see phase 3)
- [x] Compile-test the reverse include order — `test_gpsd_raw_include_order.cpp`, its own target. This is what found the `SET_HIGH_BIT` collision
- [x] Extend to Tier B — 136 messages, 15 stems; verified building and testing against all ten API pairs
- [ ] Extend to Tier C
- [ ] Decide whether `ros1_ros2_mapping.yaml` needs entries (probably not — no ROS 1 counterpart exists)

### Phase 3 — Parsers (`gpsd_client`)  *(complete for Tier A)*

- [x] `gpsd_client/include/gpsd_client/gpsd_raw_message.hpp` — the `GpsdRawMsg` alias ladder (D7). **Generated**, not hand-written, so it cannot drift from `REFERENCE_REVS`
- [x] `GpsdRawParser`: `parseRaw(const gps_data_t&, const rclcpp::Time&) -> GpsdRawMsg`
- [x] Per-pair implementations — these *are* the generated `fill()` headers, so there is one hand-written parser class rather than ten. What it adds is the part a generator cannot do safely: the ROS header, and `skyview[]`, whose length lives in a sibling field
- [x] `GpsdParserFactory::createRaw()`
- [x] `#error` below API 9; `#warning` + newest above the newest known pair
- [x] Header include order verified — see the finding below

Two things surfaced that changed the design:

**The fill guards had to become overridable.** They were `#if GPSD_API_MAJOR_VERSION == M && ... == m`, which makes the "newer than tested, fall back to the newest" policy impossible: on a future API 17 build every fill header would compile itself out. They now test `GPSD_RAW_FILL_MAJOR/MINOR`, which the ladder sets and which default to the build's own version so a header stays usable standalone.

**`SET_HIGH_BIT` was a real bug, and D9's stated rule did not cover it.** The claim in D9 that "gps.h defines no `SET_*` macros" was wrong: it defines exactly one, `SET_HIGH_BIT`. Emitting a message constant of that name broke the build the moment `gpsd_client` included a generated message — and the D9 test had an explicit carve-out for that very name, so it passed. Fixed three ways: the constant is emitted as `SET_HIGHEST_BIT`; the generator now checks **every** emitted constant against the full `#define` namespace of that rev's `gps.h` and fails loudly; and the test compares against the real macro set instead of a name pattern with an exception.

Two limits worth knowing, both now encoded in tests:

- **The legacy messages remain include-order sensitive.** `GPSStatus` defines `STATUS_RTK_FIX = 19` and `gps.h` defines `STATUS_RTK_FIX = 3`. That is a pre-existing conflict in the released `gps_msgs`, unfixable without changing published constants, and it is why `gpsd_parser.hpp:7-9` mandates an include order. D9 makes the *generated* `GPSDRaw` family order-independent; it does not and cannot fix the neighbours. `test_gpsd_raw_include_order.cpp` is a separate translation unit precisely so it can prove the generated half without dragging in the legacy half.
- **`SET_HIGHEST_BIT` may exceed the build's `SET_HIGH_BIT`.** It is a count, not a bit. `GPSDRaw14v0` is generated from gpsd 3.26.1 (45) but gpsd 3.24 reports the same API 14.0 with 44, since `EOF_SET` landed mid-pair. The test asserts `>=`, not `==` — caught by running against 3.24, where an `==` assertion failed.

### Phase 4 — Publishing (`client.cpp`)  *(complete for Tier A)*

- [x] Declare `publish_gpsd_raw` parameter (default `false`)
- [x] Create the `gpsd_raw` publisher **and** the raw parser only when enabled; the publisher handle doubles as the enabled flag
- [x] Populate and publish in `step()` alongside the existing publishes, from the same report so subscribers can align by timestamp
- [x] Add `publish_gpsd_raw` to [gpsd_client/config/gpsd_client.yaml](../gpsd_client/config/gpsd_client.yaml)
- [x] Confirm the built-in default matches the config file
- [x] Node logs which message it selected at startup (`GPSD_RAW_MESSAGE_NAME`), so an operator does not have to infer it
- [x] README: parameter row plus a "Raw gpsd messages" section

Deliberately *not* gated on `check_fix_by_variance`: that filter exists to hide
gpsd's stale-fix behaviour from `NavSatFix` consumers, and applying it here
would make a topic called "raw" a filtered one.

**Verified against a live daemon**, not just compiled. A real gpsd 3.27.5 was
built (`gpsd=True`), fed a recorded receiver log over TCP the way gpsfake's
`FakeTCP` does, and the component loaded into a real container with
`publish_gpsd_raw:=true`:

```
[INFO] [gpsd_client]: Publishing raw gpsd reports on ~/gpsd_raw as GPSDRaw16v1 (libgps API 16.1)

skyview entries            : 10
satellites_visible / used  : 10 / 5
len(skyview)==visible      : True      # trimmed, never MAXCHANNELS(184)
lat/lon  mode              : 37.293628 / -121.918752  mode=3
fix.time (timespec->Time)  : 1639082573.800000000
set mask                   : 0x1700205bffc  LATLON=True SATELLITE=True AIS=False
sat prn=  9 el= 46.0 az= 310.5 ss= 44.4 gnssid=0 used=True
```

One thing that looked like a bug and was not: with `ublox-zed-f9p_hpg1.11.log`
every message had an empty `skyview` while `satellites_used` sat at 12. That
log's SKY reports are DOP-only — `{class, device, hdop, uSat}`, no `satellites`
array and **no `nSat`** — and libgps responds by clearing `skyview` and
`SATELLITE_SET`. The 12 is not stale: gpsd parses the report's `uSat` field
directly into `satellites_used`, and that log never sends a full `SKY` at all.
(On gpsd older than 3.26.1, which has no `uSat`, the same report *would* leave a
stale value, because libgps returns before resetting it.) The raw message was
faithfully mirroring gpsd. It is also a live confirmation of the
`nSat` behaviour found while building the tier-1 harness (1.7). Switching to
`sirf2.log`, whose SKY carries `nSat` and a satellites array, produced the
output above.

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

### Phase 6 — CI  *(workflows done; awaiting a real CI run)*

- [x] Teach `tools/test_against_gpsd.sh` to accept a commit SHA as well as a release tag — `resolve_rev()` maps a bare version to `release-<v>` and passes anything else through as a commit-ish, which is what 9.1, 10.1 and 13.0 need
- [x] Read `GPSD_API_MINOR_VERSION` too — `api_version()` now returns the full pair, which matters because 9.0 and 9.1 select *different* message types
- [x] Report the selected raw message in the summary (`raw_message_for_api()`)
- [x] Run the tests via `colcon test` instead of invoking binaries by hand. The `APPEND_LIBRARY_DIRS` added in phase 5 makes this work, and running them the way a user would is the point — the old hand-rolled `LD_LIBRARY_PATH` was masking a real failure
- [x] Expand the matrix to one entry per API pair, keyed on the same reference revision the generator uses
- [x] **One workflow per API pair**, not a matrix. `gpsd_api_9v0.yml` … `gpsd_api_16v1.yml` each call the reusable [gpsd_api_shared.yml](../.github/workflows/gpsd_api_shared.yml), so the logic exists once while each pair gets its own name, badge, run history, re-run button and failure notification. A failure names the API version everywhere it appears, with no matrix row to decode
- [x] The per-version files are **generated** from `REFERENCE_REVS`, like the messages — adding an API pair creates its workflow, and `--check` fails if the tree drifts. Two tests assert one workflow per pair, calling the shared workflow, pinned to the same revision the message came from
- [x] Fast checks split into [gpsd_generator.yml](../.github/workflows/gpsd_generator.yml) — drift plus generator tests, needing only a `--filter=blob:none` clone and no gpsd build, so it runs on every push and PR
- [x] Build/test logs uploaded as `logs-api-<version>` on failure only
- [x] Each per-version workflow takes `workflow_dispatch`, so one pair can be re-run after a fix without rebuilding ten copies of gpsd; PR triggers are path-filtered so a docs-only change does not rebuild gpsd from source
- [x] README carries a badge table, one row per API pair
- [x] Add a per-job assertion that the built libgps reports the API pair the matrix claims, and that it maps to the expected `GPSDRaw<M>v<m>`
- [x] Cache key bumped to `-v3` (the matrix now keys on API pairs and includes SHAs)
- [x] **Build and test against all ten pairs locally** — libgps built at every reference revision and the full suite run against each. This is what the matrix will do on CI, and it is how the API-13 `nSat` bug below was found
- [ ] **Verify on real CI** — the workflow itself has not run on GitHub
- [ ] Watch job count/time: ten source builds of gpsd. The heavy job is already gated to `ros2-devel` pushes; the new `generator` job runs on every push and PR, which is the fast feedback path

Matrix:

All ten verified locally, each selecting the message it should:

```
9.0    GPSDRaw9v0     92 tests, 0 failures      12.0   GPSDRaw12v0    92 tests, 0 failures
9.1    GPSDRaw9v1     92 tests, 0 failures      13.0   GPSDRaw13v0    92 tests, 0 failures
10.0   GPSDRaw10v0    92 tests, 0 failures      14.0   GPSDRaw14v0    92 tests, 0 failures
10.1   GPSDRaw10v1    92 tests, 0 failures      16.0   GPSDRaw16v0    92 tests, 0 failures
11.0   GPSDRaw11v0    92 tests, 0 failures      16.1   GPSDRaw16v1    92 tests, 0 failures
```

**API 13.0 failed on the first run**, with 8 failures — the only pair that did,
and one of the three that ships in no release. The tier-1 harness emitted the
SKY `nSat` key only for API >= 14, but `nSat` landed mid-API-13 (see 1.7), so
against the end of API 13 libgps discarded every satellite in the fixture. The
threshold now keys on what libgps *tolerates* rather than when the key became
meaningful: emit `nSat` from API 11, where the catch-all `t_ignore` entry
arrived, so it is harmlessly skipped by versions that do not know it and
present for every version that does. That rule holds across both halves of API
13 and every other in-between state.

Worth stating plainly: seven of these ten pairs had never been compiled before
this sweep. Testing three of them and extrapolating would have shipped that bug.

| Workflow | API pair | gpsd revision | Message |
|---|---|---|---|
| `gpsd_api_9v0.yml` | 9.0 | `3.20` | `GPSDRaw9v0` |
| `gpsd_api_9v1.yml` | 9.1 | `e5279ef52` | `GPSDRaw9v1` |
| `gpsd_api_10v0.yml` | 10.0 | `3.21` | `GPSDRaw10v0` |
| `gpsd_api_10v1.yml` | 10.1 | `42f816d59` | `GPSDRaw10v1` |
| `gpsd_api_11v0.yml` | 11.0 | `3.22` | `GPSDRaw11v0` |
| `gpsd_api_12v0.yml` | 12.0 | `3.23.1` | `GPSDRaw12v0` |
| `gpsd_api_13v0.yml` | 13.0 | `264e808c6` | `GPSDRaw13v0` |
| `gpsd_api_14v0.yml` | 14.0 | `3.26.1` | `GPSDRaw14v0` |
| `gpsd_api_16v0.yml` | 16.0 | `3.27.3` | `GPSDRaw16v0` |
| `gpsd_api_16v1.yml` | 16.1 | `3.27.5` | `GPSDRaw16v1` |

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
| 2026-08-17 | 2 | **Tier C generated, and the messages moved to a new `gps_extended_msgs` package** (D13), keeping the `GPSD` message prefix, after measuring Tier C at 8m48s — too much to impose on the released `gps_msgs`. Tier C needed six new generator capabilities: inline *tagged* structs, enums, unions with a declarator, struct typedefs, `isgps30bits_t`, and a function-pointer test that was misfiring on a parenthesised array extent. Union arms are 0-or-1 arrays. Also D14 (rosidl name normalisation) and D15 (orphan removal), both found by breaking the build. |
| 2026-08-17 | 2/3/4 | **Tier B landed.** 136 generated messages (was 64), 7 new sub-messages. Parser gained `devices.list` (trimmed to `ndevices`) and `imu[]` (terminated by an empty `attitude_t::msg`, the rule gpsd's own dumper uses — there is no count field). New exclusion: pointer members, by type not name. Completeness cross-check and manual expectations extended to all Tier B structs. `colcon test`: **100-102 tests, 0 failures on all ten API pairs**. |
| 2026-08-17 | — | D12: fixed a live dangling-pointer defect in `client.cpp` found while excluding `fixsource_t`'s pointers. Confirmed with ASAN (`stack-use-after-return` before, clean after). |
| 2026-08-16 | 6 | Replaced the matrix with one generated workflow per API pair calling a shared reusable workflow, so each version has its own name, badge and re-run. Generated from `REFERENCE_REVS` and covered by two new drift tests; README gained a per-version badge table. |
| 2026-08-16 | 6 | Phase 6: CI matrix expanded to all ten API pairs plus a fast generator-only job; `test_against_gpsd.sh` now accepts commit SHAs, reports the full API pair, and runs via `colcon test`. Built libgps at all ten reference revs and ran the suite against each — **92 tests, 0 failures on every pair**. Found and fixed a real bug doing so: the `nSat` threshold was wrong for API 13 (see 1.7). |
| 2026-08-16 | 4 | Phase 4 complete for Tier A: `publish_gpsd_raw` (default false), opt-in publisher + parser, config and README. Verified end to end against a real gpsd daemon fed a recorded receiver log — 10 satellites published, `skyview` trimmed to `satellites_visible`, mask and NaN semantics intact. |
| 2026-08-16 | 3 | Phase 3 complete for Tier A: generated selection ladder, `GpsdRawParser` (header + clamped `skyview[]` fill), `createRaw()`. Found and fixed a real D9 bug — `SET_HIGH_BIT` is a gps.h macro, so the emitted constant broke the build; generator now checks the whole macro namespace. Fill guards made overridable so the newer-than-tested fallback can work. `colcon test`: **92 tests, 0 failures** on gpsd 3.20/3.24/3.27.5. |
| 2026-08-16 | 1/5 | Generated-output tests added (30) and all Python tests wired into `colcon test` via `gps_msgs` + `ament_add_pytest_test`; drift check now runs as an ordinary test. Fixed `colcon test` failing to find a source-built libgps by adding `APPEND_LIBRARY_DIRS` to both gtest targets. Whole workspace: 77 tests, 0 failures, 0 skipped on gpsd 3.20/3.24/3.27.5. Mutation-tested the suite: 12/12 injected generator bugs caught. |
| 2026-08-16 | 1 | Generator implemented for Tier A: 70 messages + 10 parser headers + `gpsd_has_member.hpp`, 23 unit tests, `--check` verified both ways. `gps_msgs` builds all of them; generated fill code verified running against real libgps; D11 confirmed against gpsd 3.24. Corrected the emitted `#include` paths after checking them against rosidl's actual output. |
| 2026-08-16 | 0 | Phase 0 complete. Upstream refetched (no new tags). Two corrections: `MAXCHANNELS` released values are 140/184, not 140/185/230; and API pairs span *ranges* of header states, not single states — `gps_fix_t` grows by six members within API 14.0, and 16.1 covers both 184 and 230. Added D11 (C++ member detection, validated on three installs) to handle it. Reference revs pinned and all ten verified in [tools/generate_raw_msgs.py](../tools/generate_raw_msgs.py). |
| 2026-08-16 | 5 | Tier-1 harness landed and verified on API 9.0/14.0/16.1. Found three libgps changes that do *not* track the API version (new section 1.7) — `gps_unpack()`'s constness, `gps_clear_gst()`'s existence, and the SKY `nSat` requirement. Also confirmed no released gpsd propagates parse errors out of `gps_unpack()`, so fixture typos surface as value assertions, never as a parse failure. |
| 2026-08-16 | 5 | Test data strategy added (new section 5; later sections renumbered). Two tiers: `gps_unpack()` for per-field coverage on all ten versions, gpsfake for end-to-end on the newest. Completeness is enforced by a generated header-audit test, not by hand-written cases. |
| 2026-08-16 | — | D10: AIS excluded. No `ais` union arm, no `GPSDAis*` sub-message, no `ais_t` support in the generator. `SET_AIS` is still emitted and `set` still carries the bit, so an omitted AIS report stays detectable. |
| 2026-08-16 | — | gpsd 3.27/3.27.1/3.27.2 (`api_version_major == 0`) declared out of scope. No coverage lost — all three are C API 16.0, already represented by `release-3.27.3`. Dropped the phase 7 task to document the quirk. |
| 2026-08-16 | — | D9 decided: emit the `set` bitmask constants. Renamed `<NAME>_SET` → `SET_<NAME>` because rosidl emits constants as `static constexpr` members and `gps.h` defines the gpsd spellings as global macros. Open question 3 closed. |
| 2026-08-16 | — | Plan created. API↔release table derived from gpsd git history; API 15 confirmed never to have existed. |
