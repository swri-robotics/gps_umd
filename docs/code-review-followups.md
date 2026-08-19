# Code review follow-ups

Working document for the changes coming out of the offline code review of the
raw GPSd message work.

**Lifecycle:** this is a working file, not reference documentation. Delete it
before merging, the same way the design plan was. Anything here that turns out
to be a durable fact about GPSd or the message layout belongs in
[gpsd-quirks.md](gpsd-quirks.md) or
[gpsd-raw-message-structure.md](gpsd-raw-message-structure.md) instead, and
should be moved there rather than left to disappear with this file.

---

## Status

| | |
|---|---|
| Findings recorded | 2, both closed |
| Decisions settled | 3 of 3 |
| Implemented | **both findings complete** |
| Premise checks passed | 1 of 1 |
| Verified | both findings: 12/12 sweep + end-to-end, 0 failures |
| Branch | `per_api_version_messages` |
| Baseline | `8d6147a`, clean, in sync with `origin` |

Baseline is green: `--check` clean at 927 generated files, 65 generator tests,
74 end-to-end tests with 0 failures and 0 skips, twelve sweep labels passing.

### Headline numbers

Measured against the current tree, not estimated.

| | now | after finding 2 | after both |
|---|---|---|---|
| `.msg` files | 905 | 221 | **10** |
| Message families | 100 | 24 | 1 |
| Fields in `GPSDRaw16v1` | 32 top level | 32 | 274 flat |
| Fields in `GPSDRaw9v0` | 30 top level | 30 | 166 flat |

**RTCM and subframe are deleted, not flattened.** Finding 2 removes those
message trees entirely and replaces them with the raw GPSd JSON string on its
own topic. Finding 1 then flattens only what remains. Because finding 2 runs
first, no RTCM or subframe message is ever flattened — they are gone before the
flattening work starts.

**Do finding 2 first.** It removes 684 of the 905 `.msg` files (76%) on its own,
and it makes finding 1 dramatically smaller: the flattened root drops from 514
fields to 274, and the awkward nested-array cases drop from 11 to 1. Flattening
first would mean flattening ~700 files that the next step deletes.

---

## Findings

### 1. Too many messages; flatten the structure

**Raised:** `gps_extended_msgs` carries far too many message types and is
confusing to navigate. Keep the per-API-version raw messages, but remove nested
project-defined messages — hoist their fields to the top level of
`GPSDRaw<M>v<N>`, and turn arrays of nested messages into parallel arrays of
their scalar fields. Only standard ROS types should remain nested.

**Assessment:** Agree on the problem, and it is structurally achievable — but it
is not free, and two consequences need an explicit decision before any code
changes.

*It works everywhere.* I walked all ten roots looking for the one shape that
cannot flatten — an array of structs that itself contains an array, which would
need a 2D array ROS does not have. **There are none.** Every intermediate level
that carries an array is either a single struct (`devices`) or a 0-or-1 union
arm (`raw`, `rtcm3`, `subframe`), so every leaf array collapses to a flat
parallel array. That holds even for the deepest chain in the tree,
`rtcm3 → rtcmtypes → rtcm3_msm → sat[]`, which bottoms out in scalars — checked
only to prove no shape in `gps_data_t` blocks flattening. RTCM itself is deleted
by finding 2 and never reaches this step.

*Names must carry their path.* Flattening to bare leaf names produces **63
collisions** in `GPSDRaw16v1` alone — `time`, `status`, `alt_hae`, `temp`,
`vel_n` and more each occur in several sub-messages. Path-prefixed names
(`fix_time`, `gst_time`, `skyview_time`) give 0 collisions. So the scheme is
forced: `<path>_<leaf>`, joined with `_`.

*The two costs.* Neither blocks the work; both should be accepted knowingly.

- **Parallel arrays drop a guarantee the type system currently makes.** Today
  `skyview[i]` is one satellite. Flattened, `skyview_prn[i]` and
  `skyview_azimuth[i]` are correlated only by convention, and nothing stops them
  having different lengths. Worth a generator invariant plus a test.
- **The root message gets large** — 166 to 274 fields depending on API version.
  That is the point of the change (one type instead of 100), but it will read as
  a wall of fields, and `ros2 topic echo` output becomes long.

**Scope:** `tools/generate_raw_msgs.py` (emission and the fill code), all 927
generated files, the tests in `tools/test_generated_messages.py` that assert
sub-message shape, and both reference docs.

**Decisions** *(settled 2026-08-19)*

1. **Names carry their path**: `<path>_<leaf>`, joined with `_` — `fix_time`,
   `skyview_elevation`, `raw_meas_svid`. Forced rather than chosen: bare leaf
   names collide 63 times in `GPSDRaw16v1`, path-prefixed collide 0 times.

2. **Union arms flatten to plain scalars**, not to 0-or-1 arrays. `version`,
   `osc`, `raw` and `error` — 11 fields — become scalars, and `set` says which
   arm is live, which is already the documented contract. `string error`
   instead of `string[] error` reads the way a caller expects.

   **This retires the 0-or-1 array convention entirely**, so the structure doc
   section describing it goes, and the mask stops being optional for anyone
   reading a union arm.

3. **Length consistency is structural, plus a test.** The generated fill
   resizes and writes every array in a group from *one* loop, so unequal
   lengths are unrepresentable rather than merely tested for; a generated test
   backs it up.

Four real arrays become parallel arrays. Group sizes in `GPSDRaw16v1`:

| Group | Parallel arrays | Length |
|---|---|---|
| `imu_*` | 35 | entries before the first empty `msg` |
| `raw_meas_*` | 16 | `meas[]` entries with a usable `svid` |
| `skyview_*` | 14 | `satellites_visible` |
| `devices_list_*` | 14 | `ndevices` |

Result: 274 fields in `GPSDRaw16v1` — 79 arrays, 195 scalars — and 166 in
`GPSDRaw9v0`.

- [x] Naming scheme decided: `<path>_<leaf>`
- [x] Array-length consistency decided: one loop + generated test
- [x] Generator: `flatten_model()` collapses the nested messages into one
      flat root. **905 → 11 `.msg` files** across both findings
- [x] Generator: fill rewritten as a tree over the C paths, so `fix.ecef.x`
      and `fix.ecef.y` share their guards; plus one filler per array group
- [x] `test_generated_messages.py` reworked — 20 of its tests asserted the
      nested shape. Completeness now maps each struct to a field *prefix*
      and matches with `covers()`, since snake_case names contain
      underscores and a flat name cannot be split back into segments
- [x] Regenerate: 11 `.msg` (10 `GPSDRaw` + `GPSDJson`), `--check` clean at
      33 files. `GPSDRaw16v1` is 274 fields: 79 arrays, 195 scalars
- [x] Full 12-label sweep — all 12 PASS, 60-62 tests each, 0 failures.
      The 20 end-to-end tests skip on the libgps-only labels, which is
      correct: they need a full GPSd build
- [x] Structure doc rewritten: new diagram, the 0-or-1 array section replaced
      by the mask-gated scalar rule, a parallel-array section with the four
      groups, and the version table rebuilt as field counts per version

### 2. RTCM is overkill; publish the raw GPSd JSON instead

**Raised:** the RTCM message tree is disproportionate. Check whether we can
publish the raw GPSd JSON string and let consumers parse RTCM (and subframe)
themselves.

**Assessment:** Agree, and this is the higher-value change of the two. RTCM and
RTK are **564 of 905** `.msg` files; the subframe family is another **120**.
Together they are 76% of the package for data almost no subscriber decodes.

**It is also strictly more capable than what we have now, which is the part
worth pausing on.** `gps_read()` copies the JSON line into the caller's buffer
*before* `gps_unpack()` runs:

```c
message_len = 1 + eol - PRIVATE(gpsdata)->buffer;
if (NULL != message) {
    memcpy(message, PRIVATE(gpsdata)->buffer, message_len);
}
status = gps_unpack(PRIVATE(gpsdata)->buffer, gpsdata);   // may ignore the class
```

libgps decodes only 17 report classes and silently drops the rest, which is why
`subframe` and `log` can never populate through the struct — see
[gpsd-quirks.md](gpsd-quirks.md). But the raw line is captured regardless of
whether `gps_unpack` understands it. So publishing the JSON would make SUBFRAME
data reachable **for the first time**, where the current typed path cannot
deliver it at any API version.

The node already has this string: `client.cpp` reads it into `message_` and
parses the class out of it to route RTCM. Publishing it is close to free.

**Decisions** *(settled 2026-08-18)*

1. **Message type: a `Header` plus the string.**

   ```
   # gps_extended_msgs/msg/GPSDJson.msg
   std_msgs/Header header   # stamp = system clock when the report was read
   string json              # one GPSd JSON report, verbatim
   ```

   **This is the first message with no version suffix, and that is correct.**
   The string is opaque, so its shape does not vary with the GPSd API — one type
   serves all ten pairs. Worth stating in the structure doc, since every other
   message is version-keyed and a reader will expect `GPSDJson16v1`.

2. **Publish every report class.** No filtering by class, so TPV and SKY appear
   on this topic as well as in `GPSDRaw`. Simple rule, nothing to document as an
   exception.

3. **`gpsd_rtcm2` / `gpsd_rtcm3` disappear immediately.** No deprecation period —
   the package has not merged to mainline, so there are no subscribers to break.

4. *Consequence, not a decision:* `subframe` leaves `GPSDRaw` with RTCM, so the
   structure doc must say the message mirrors the **reachable** parts of
   `gps_data_t`, with the rest available as JSON.

### What these decisions delete

Decision 2 removes the reason the node inspects the JSON at all. Nothing needs
the report class once every class is published, so the routing logic goes:

| Removed | Where |
|---|---|
| `reportClass()` | `client.cpp` — existed only to route RTCM |
| `publishRtcm()` | `client.cpp` |
| `parseRtcm2()`, `parseRtcm3()` | `gpsd_raw_parser.{hpp,cpp}` |
| `publish_gpsd_rtcm` parameter | `client.cpp`, config, README |
| 11 RTCM unit tests | `test_gpsd_raw_parser.cpp` |

Decision 1 also simplifies timestamping. The fix topics take one `now` per
publish cycle, but JSON is per report and several arrive in one cycle, so each
message stamps at its own read rather than sharing the cycle's value — which is
what "when the JSON was received" means. Take it from the node clock so
`use_sim_time` still behaves.

**Scope:** `client.cpp`, the generator's exclusion lists, ~684 generated files,
the RTCM end-to-end tests, and both reference docs.

- [x] **Premise verified** (2026-08-18). Replayed `ublox-ned-m8t-sbfrx3` through
      gpsfake into a real GPSd 3.27.5 and read it with `gps_read()` exactly as
      the node does. Of **443** JSON lines returned to the caller's buffer,
      **438 carried `"class":"SUBFRAME"`** — complete with the decoded `ALMANAC`
      payload — while `SUBFRAME_SET` appeared in `gps_data_t::set` on
      **0** reports. The JSON path delivers everything the struct path cannot
- [x] Add `GPSDJson.msg` (unversioned) and the `publish_gpsd_json` parameter —
      **emitted by the generator, not checked in by hand**: `orphans()` owns
      the whole `msg/` directory, so a hand-written file there is deleted by
      the next regenerate
- [x] Publish one message per report inside the drain loop, stamped at read
- [x] Delete `reportClass`, `publishRtcm`, `parseRtcm2/3`, `publish_gpsd_rtcm`
- [x] Remove RTCM, RTK and subframe from the generator's scope entirely.
      Also deleted the dispatch machinery they needed (`emit_rtcm2/3_dispatch`,
      `emit_subframe_dispatch`, `RTCM2_TYPE_ARMS`, `RTCM3_MSM_RANGES`,
      `SUBFRAME_ARMS`, `SUBFRAME_PAGE_ARMS`, `STANDALONE_ROOTS`) — verified
      genuinely dead by regenerating: **zero** change to any output file
- [x] Replace the RTCM end-to-end tests with JSON-topic equivalents (5 tests,
      including one asserting SUBFRAME present on `gpsd_json` *and* absent
      from the typed mask), and 14 RTCM/subframe unit tests deleted
- [x] Regenerate: **905 → 177 `.msg` files**, 100 → 20 families, `--check`
      clean at 199 generated files. No orphans: all 177 reachable from a root
- [x] Full 12-label sweep — all 12 PASS, 58-60 tests each, 0 failures.
      The 20 end-to-end tests skip on the libgps-only labels, which is
      correct: they need a full GPSd build
- [x] Update both reference docs, README and the config yaml. The §1.9 entry
      now says the JSON topic sidesteps it, with the 438/443 vs 0 measurement

---

## Sequencing

1. Verify the SUBFRAME-in-JSON premise (finding 2's first box)
2. Settle finding 2's four open decisions, then implement it — 905 files → 221
3. Settle finding 1's two decisions, then implement it — 221 files → 10
4. Full sweep, then rewrite both reference docs once, against the final shape

Docs last: rewriting them between steps 2 and 3 means writing them twice.

---

## Ground rules

Constraints that already cost time once. Check a change against these before
writing it.

- **Never hand-edit generated files.** 927 files come from
  `tools/generate_raw_msgs.py` — every `.msg`, every fill header, and the ten
  `gpsd_api_*.yml` workflows. Edit the generator, then regenerate. `--check`
  catches drift.
- **Never hand-edit `CHANGELOG.rst`.** A separate release tool owns those.
- **Probe, never compare versions.** One API pair spans a range of header
  states. Use the `has_<member>` traits in C++, or `check_struct_has_member` /
  `check_cxx_symbol_exists` in CMake when the preprocessor needs the answer.
- **A passing test proves nothing until it can fail.** Several bugs here
  produced green, successful-looking runs. Mutate the code and confirm the test
  goes red before believing it.
- **Watch for skips.** The end-to-end suite skips nothing by design now, so any
  skip is a signal. Read the count, not just the tick.
- **Heavy CI runs on `ros2-devel` pushes and on pull requests only.** Pushing
  this branch no longer triggers the per-API or end-to-end workflows.

## Verification

Run from the package root with a ROS environment sourced. Times are for a warm
cache.

| Check | Command | Expected |
|---|---|---|
| Generator drift | `python3 tools/generate_raw_msgs.py --check` | `927 generated files up to date` |
| Generator + message tests | `python3 -m pytest tools/test_generate_raw_msgs.py tools/test_generated_messages.py -q` | `65 passed`, ~2s |
| One API version | `tools/test_against_gpsd.sh 3.27.5` | `PASS`, ~1 min |
| End to end | `GPSD_FULL_BUILD=1 tools/test_against_gpsd.sh 3.27.5` | `74 tests, 0 failures, 0 skipped` |
| Full sweep (12 labels) | see below | all `PASS`, ~25 min cold |
| Python lint | `python3 -m flake8 --max-line-length=99 tools/*.py gpsd_client/test/*.py` | no *new* warnings — diff against a baseline, several are pre-existing |
| Shell | `bash -n tools/test_against_gpsd.sh` | silent |
| Workflows | `for f in .github/workflows/*.yml; do python3 -c "import yaml; yaml.safe_load(open('$f'))"; done` | silent |

The expected counts above are the **baseline**; both findings change them.
Update this table as they land.

Full sweep — ten reference revisions plus the two non-reference revs that
bracket changes no version comparison can express:

```bash
tools/test_against_gpsd.sh \
  3.20 e5279ef52 3.21 42f816d59 3.22 3.23.1 264e808c6 3.24 3.25 3.26.1 3.27.3 3.27.5
```

### Environment note

`.gpsd_versions/` caches source-built libgps and the message packages, and both
are **specific to the container** — they embed a glibc version and a ROS
distribution. After a container or image change, clear them or every build fails
with undefined `glibc` symbols or a missing `rosidl` target:

```bash
rm -rf .gpsd_versions/install .gpsd_versions/msgs .gpsd_versions/colcon
```

Keep `.gpsd_versions/gpsd` — it is a git clone and ports fine.

---

## Session log

Newest first. One row per working session: what changed, and anything learned
that the next session would otherwise rediscover.

| Date | Note |
|---|---|
| 2026-08-19 | **Finding 1 complete.** Full 12-label sweep all PASS (60-62 tests per version, 0 failures). Counts still rise monotonically with API version. Net across both findings: **905 → 11 `.msg` files**, 100 → 1 message family, full build ~8 min → ~2 min. |
| 2026-08-19 | **Finding 1 implemented.** 177 → 11 `.msg`; `GPSDRaw16v1` is one flat message of 274 fields. Three things the measurements did not predict: (1) union arms became scalars but still need the **mask gate in the fill** — the C members share storage, so reading an unnamed arm is still a read of an inactive union member; (2) `raw_meas` is a *filtered* subset (GPSd marks unused entries with svid 0/255), not a prefix, so the group fillers take **source indices** rather than a count — which also covers the prefix groups uniformly; (3) flattening created `skyview_time`, a per-report scalar sitting among the per-satellite `skyview_*` arrays — not a collision but confusable, so it is called out in the docs and special-cased in the completeness test. End-to-end green: 62 tests, 0 failures, 0 skipped. |
| 2026-08-19 | **Finding 2 complete.** Full 12-label sweep all PASS (58-60 tests per version, 0 failures; the 20 end-to-end tests skip without a full GPSd build, which is correct for the libgps-only labels). Counts still rise monotonically with API version, the weak check that each build really compiled against its own header. |
| 2026-08-19 | **Finding 2 verified end to end**: 60 tests, 0 failures, **0 skipped**, including the 5-test `JsonTopic` suite. One stale test caught it — `test_subframe_arm_stays_empty` asserted a field that no longer exists and failed with `AttributeError`. Reworked rather than deleted: the class is now `LogIsUnreachableThroughLibgps` (LOG really is still unreachable and the field remains), the SUBFRAME premise guard moved into `JsonTopic` so its assertions cannot pass against a log with no subframes, and the log test shares `JsonTopic`'s capture instead of replaying the same 15s log twice. |
| 2026-08-19 | **Finding 2 implemented.** 905 → 177 `.msg` files, 100 → 20 families; `gpsd_json` publishes every report per read. Build 8min → 2min. Three things worth carrying forward: (1) `GPSDJson.msg` must be *generated*, since `orphans()` owns all of `msg/` and would delete a hand-written file; (2) the RTCM/subframe dispatch machinery was provably dead — deleting it changed no generated byte; (3) `tools/test_against_gpsd.sh` caches `.gpsd_versions/msgs/install` with **no staleness check against the `.msg` files**, so it silently reused a pre-`GPSDJson` build and failed with a missing header. CI is unaffected (it does not cache that path), but clear it by hand after changing the message set. |
| 2026-08-18 | **SUBFRAME-in-JSON premise confirmed empirically.** gpsfake replay of `ublox-ned-m8t-sbfrx3` into GPSd 3.27.5, read through `gps_read()`: 443 lines reached the caller's buffer, 438 of class SUBFRAME with full ALMANAC payloads, and `SUBFRAME_SET` was never set on `gps_data_t` — 0 of 443. So the JSON topic exposes data the typed path cannot reach at any API version, and finding 2 is a capability gain rather than only a simplification. Probe kept out of the repo; it is ~40 lines of C against libgps. |
| 2026-08-18 | All three open decisions settled: `GPSDJson` = `Header` + `string`, stamped per report at read time and **unversioned** (the string is opaque, so it does not vary by API pair — the only such message); publish every report class, no filtering; RTCM topics removed outright, no deprecation, since nothing downstream has merged. Publishing every class removes the need to inspect the JSON, so `reportClass()`, `publishRtcm()`, `parseRtcm2/3()`, the `publish_gpsd_rtcm` parameter and 11 RTCM unit tests all go. |
| 2026-08-18 | Both findings recorded with measurements. Flattening confirmed structurally possible everywhere — no array-of-struct-containing-an-array exists, because every intermediate is a single struct or a 0-or-1 union arm. Bare leaf names collide 63 times, so path-prefixed names are forced. RTCM/RTK plus subframe are 684 of 905 `.msg` files, so finding 2 shrinks finding 1 by roughly half (514 flat fields → 274). Established that `gps_read()` copies the JSON line before `gps_unpack()`, so a JSON topic would expose SUBFRAME, which the typed path cannot reach at any version — pending empirical confirmation. |
| 2026-08-18 | Document created. Baseline `8d6147a` green on all checks. |

---

## Template for a finding

```markdown
### N. <short title>

**Raised:** <what the review said>
**Assessment:** <agree / disagree / partly, and why — a review item may be
wrong, and saying so with evidence is a valid outcome>
**Scope:** <files and whether the generator is involved>

- [ ] Change
- [ ] Test that fails without the change
- [ ] Verification run (which of the checks above, and the result)

**Notes:** <anything discovered along the way>
```
