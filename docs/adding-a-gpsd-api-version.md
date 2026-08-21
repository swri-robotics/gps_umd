# Adding a new GPSd API version

GPSd bumps `GPSD_API_MAJOR_VERSION` / `GPSD_API_MINOR_VERSION` every year or
two, and each bump needs a new `GPSDRaw<MAJOR>v<MINOR>` and its parser. This is
the procedure. It is short because the generator does most of it; the parts
that need judgement are called out, and every one of them has bitten this
package at least once.

Read [gpsd-raw-message-structure.md](gpsd-raw-message-structure.md) and
[gpsd-quirks.md](gpsd-quirks.md) first if you have not — this document assumes
the message layout and the GPSd behaviours they describe.

## 0. The one thing to internalise

**An API pair names a *range* of header states, not one state.** GPSd bumps the
version when a breaking change begins, then keeps adding fields under the same
number until the next bump. So "API 13" is not a fixed `gps.h`; `nSat` is absent
at the start of API 13 and present at the end of it.

Every consequence in this document follows from that. In particular: a feature
test of the form "API >= 13, therefore field X exists" is *wrong* unless you
have checked the specific revision. Use `if constexpr` member detection
(`gpsd_has_member.hpp`) rather than version arithmetic, in both the
generated fill code and the tests.

## 1. Find the revision

The manifest pins a *commit or tag*, not a version number, and the rule is: the
**last revision at that pair**, so the message covers everything the pair ever
grew (see step 0).

Start by seeing what each release reports. This reads one blob per tag and takes
about a second:

```bash
cd .gpsd_versions/gpsd     # the clone tools/test_against_gpsd.sh maintains
git fetch --all --tags
for tag in $(git tag -l 'release-3.*' | sort -V); do
  h=$(git show "$tag:include/gps.h" 2>/dev/null || git show "$tag:gps.h" 2>/dev/null)
  printf '%-16s %s.%s\n' "$tag" \
    "$(printf '%s' "$h" | awk '$2=="GPSD_API_MAJOR_VERSION"{print $3}')" \
    "$(printf '%s' "$h" | awk '$2=="GPSD_API_MINOR_VERSION"{print $3}')"
done
```

(`gps.h` moved into `include/` partway through GPSd's history, hence the
fallback.) If the new pair appears there, the newest tag carrying it is your
reference rev, and you are done with this step.

If the pair never shipped in a release — 9.1, 10.1 and 13.0 did not — find the
commit that bumped *past* it and take its parent. Bound the search between the
two surrounding tags; the pickaxe is instant that way and takes minutes over the
whole history:

```bash
git log --oneline --reverse -G'GPSD_API_M(AJ|IN)OR_VERSION' \
    release-3.20..release-3.21 -- include/gps.h gps.h
git rev-parse --short=9 <the-bump-commit>^
```

That is exactly how `(9, 1): "e5279ef52"` was derived: the parent of
`29991d6f`, the commit that moved `status` into `gps_fix_t` and bumped to 10.0.

Record the date and rationale in the comment, as the existing entries do —
"last commit at API 13.0" is the fact a future reader needs.

## 2. Update the manifest

Two dicts at the top of [tools/generate_raw_msgs.py](../tools/generate_raw_msgs.py):

```python
REFERENCE_REVS = {
    ...
    (17, 0): "release-3.28",
}

EXPECTED_MAXCHANNELS = {
    ...
    (17, 0): 184,          # cross-check only; never emitted
}
```

`EXPECTED_MAXCHANNELS` is a tripwire, not data. If the generator reports a
mismatch, you read a different revision than you thought — do not "fix" it by
editing the number until it agrees. Note the gps.h changelog has claimed values
(185, 230) that no release ever shipped; trust the header at the rev.

The manifest drives the messages, the parser headers *and* the CI workflow
files, so nothing else needs a version list.

## 3. Regenerate

```bash
cd <workspace>/src/gps_umd
python3 tools/generate_raw_msgs.py
git status --short          # new msg/, new parser header, new workflow
```

The generator refuses to guess. Expect it to stop with a `SystemExit` on any of:

* **an unmapped C type** — add it to the type map, with a test; silently
  dropping a member would lose data with no trace
* **a colliding ROS field name** — two GPSd members that snake_case to the same
  thing
* **an unknown `#ifdef`** — it resolves the conditionals it knows and refuses
  the rest rather than guessing which arm is live
* **a mask constant colliding with a `gps.h` macro** — this is why the
  constants are `SET_LATLON` and not `LATLON_SET`, and why `SET_HIGH_BIT`
  had to become `SET_HIGHEST_BIT`

It also *deletes* files it used to emit and no longer does. That is deliberate:
the message package globs its directory, so a stale file left behind after a
rename is built as a second, conflicting definition.

## 4. Handle new union arms by hand

New members are automatic. New *union arms* are not, because there is no way to
read from a union safely without knowing which arm is live. If the new `gps.h`
adds one, add it to the relevant dispatch table:

| Union | Table | Selected by |
|---|---|---|
| `gps_data_t`'s report union | `REPORT_UNION_BITS` | the `set` mask bit |
| `rtcm2_t` | `RTCM2_TYPE_ARMS` | `rtcm2.type` |
| `rtcm3_t` | `RTCM3_MSM_RANGES` and the type map | `rtcm3.type` |
| `subframe_t` | `SUBFRAME_ARMS`, `SUBFRAME_PAGE_ARMS` | `subframe_num` / `pageid` |

Derive the mapping from **GPSd's own writer code**, not from the arm's name.
Several arms — `rtcm2_18` through `rtcm2_24`, and `subframe_t::sub4` — are
declared in `gps.h` and written by nothing in GPSd. Mapping them by name pattern
would publish uninitialised union bytes. There are tests pinning them empty;
leave them that way unless you find the code that fills them.

## 5. Add manual expectations

The generator being self-consistent proves nothing about whether it read the
right header. `ManualExpectations` in
[tools/test_generated_messages.py](../tools/test_generated_messages.py) is the
independent check: one test per real GPSd change, each stating *what* changed
and *why* the message must reflect it.

Add at least one for the new pair — whatever the bump was actually for. If the
bump moved a field, assert both its absence before and its presence after, the
way `test_status_lives_in_gps_data_t_only_on_api_9` does. A bump with no
corresponding expectation is an untested version.

## 6. Test against the real library

```bash
tools/test_against_gpsd.sh 3.28              # builds libgps at that rev, then gpsd_client
colcon test --packages-select gps_msgs gpsd_client
```

The first builds libgps from source at the requested revision and runs
`gpsd_client`'s tests linked against exactly that library — this is what
exercises the version-specific code paths, which are preprocessed away when
building against anything else.

Watch for the failure mode that recurs: a *test* that assumes a field exists.
The fixtures and assertions have to be as version-aware as the parser. This has
been missed four separate times (`nSat`, `gps_clear_gst`, `gps_unpack`'s
`const`ness, the `rtcm2` fields at API 10) and each time it looked like a
generator bug at first.

### Why the script's toolchain is not in any `package.xml`

`rosdep install --from-paths src --ignore-src` does not give you enough to run
the script above, and it is not meant to. These are needed, and no manifest
declares them:

| Tool | Needed for |
|---|---|
| `scons` | building libgps from source — the script's own first check, so a missing `scons` fails as `error: scons is required` rather than anything subtler |
| `python3-serial`, `libdbus-1-dev` | `GPSD_FULL_BUILD=1`, which additionally builds the GPSd daemon and the `gps` Python module that `gpsfake` needs |
| `python3-dev`, `python3-numpy` | `rosidl_generator_py` building `gps_msgs`' Python bindings, which some ROS base images do not pull in transitively |
| `git` | `tools/test_generated_messages.py` reads `gps.h` at each pinned revision with `git show` |

They are build dependencies of *GPSd* and of this harness, not of these
packages' tests, and everything they enable sits behind a skip condition:
without `GPSD_E2E_PREFIX` the gpsfake suite skips itself, and without
`GPSD_REPO` the two generator suites skip themselves. What is left is
`gpsd_client`'s four gtest suites, which build against whatever libgps the
distro ships and need nothing beyond what `package.xml` declares. That is
precisely what `humble.yml` and its siblings do: plain `industrial_ci` with no
`apt-get` step at all, which is also how the ROS build farm builds this
repository. Adding `<test_depend>scons</test_depend>` would push a third-party
project's build tooling onto the farm and onto every downstream consumer, for
tests that can never run there.

**So the multi-version and end-to-end suites are CI jobs, not part of `colcon
test`.** The list lives in two places rather than a manifest: the Requirements
block at the top of `tools/test_against_gpsd.sh`, and the "Install build tools
and workspace dependencies" step in `.github/workflows/gpsd_api_shared.yml` and
`gpsd_end_to_end.yml`. Keep those in step with each other. Running the script
locally means installing them yourself first.

Both workflows also pass `rosdep install --skip-keys "libgps gpsd"`. `libgps`
matters: the key is declared, and skipping it stops rosdep installing a distro
`libgps-dev` that the script is about to override with a pinned source build.
`gpsd` is inert — no manifest declares that key — and stays only as insurance
against one ever doing so, since a system daemon of the wrong version cannot
drive `gpsfake` (see the test's docstring).

## 7. Documentation and CI

The workflow file `.github/workflows/gpsd_api_<M>v<m>.yml` is generated in step
3 — commit it. It calls the reusable `gpsd_api_shared.yml`, which also asserts
that the selection ladder picked the message you expect, so a build that
compiles but selects the *previous* version's message fails rather than passing
quietly.

Hand-maintained, so do these yourself:

* the API coverage table in [README.md](../README.md) — add a row with the
  revision, the message name and a badge pointing at the new workflow
* the version tables in
  [gpsd-raw-message-structure.md](gpsd-raw-message-structure.md) — the
  per-version message counts and any family that appears or disappears
* a note in [gpsd-quirks.md](gpsd-quirks.md) if the new pair spans a range
  where fields appeared without a bump

Do **not** touch `CHANGELOG.rst`. Those are generated from commit history by a
separate release tool; edit them by hand and your entry is clobbered or
duplicated at the next release. Write what you would have put there into the
commit message, which is what the tool actually reads.

## 8. Confirm

```bash
python3 tools/generate_raw_msgs.py --check   # exits non-zero on drift or orphans
```

CI runs this too (`gpsd_generator.yml`), so the checked-in output cannot drift
from the script. It is the last thing to run and the first thing to check when
something looks inexplicable.
