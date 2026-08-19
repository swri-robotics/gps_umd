#!/usr/bin/env python3
"""Verify the generated messages match the real gps.h for each API version.

This is the counterpart to test_generate_raw_msgs.py. That file exercises the
parser against synthetic fragments; it would happily pass while the generator
emitted a message that had nothing to do with the GPSd version it claims. These
tests close that gap in two independent ways:

1. **Manual expectations** (ManualExpectations below). Hand-written tables of
   members that must and must not appear in each API pair's messages, derived
   from the gps.h changelog and from this project's own established facts. They
   encode *why* a field appears where it does -- gps_data_t::status moving into
   gps_fix_t at API 10, leap_seconds arriving at 9.1, baseline_t at 13.0 -- so a
   regression names the specific GPSd change it broke.

2. **An independent completeness cross-check** (Completeness below). Struct
   members are re-extracted from the real gps.h by a deliberately separate,
   simple scanner, and every non-excluded member is required to have a field in
   the generated message. Reusing the generator's own parser here would be
   circular: it could drop a member and the test would agree with it.

Both need the GPSd clone, so they are integration tests and skip cleanly when
it is absent. Run with:

    python3 -m unittest discover -s tools -p 'test_*.py'
"""

import importlib.util
import os
import re
import subprocess
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_SPEC = importlib.util.spec_from_file_location(
    "generate_raw_msgs", os.path.join(_HERE, "generate_raw_msgs.py"))
gen = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gen)

# <workspace>/src/<repo>/tools/this_file -> <workspace>/.gpsd_versions/gpsd,
# the clone tools/test_against_gpsd.sh maintains.
_WORKSPACE = os.path.dirname(os.path.dirname(os.path.dirname(_HERE)))
GPSD_REPO = os.environ.get(
    "GPSD_REPO", os.path.join(_WORKSPACE, ".gpsd_versions", "gpsd"))

_HAVE_REPO = os.path.isdir(os.path.join(GPSD_REPO, ".git"))
_GENERATED = None


def generated():
    global _GENERATED
    if _GENERATED is None:
        _GENERATED = gen.generate(GPSD_REPO, gen.CHECKED_IN_SCOPE)
    return _GENERATED


def message_fields(pair, stem):
    """Field names of <package>/msg/<stem><major>v<minor>.msg, or None."""
    text = generated().get(f"{gen.PACKAGE}/msg/{stem}{pair[0]}v{pair[1]}.msg")
    if text is None:
        return None
    return [line.split()[-1] for line in text.splitlines()
            if line and not line.startswith("#") and "=" not in line]


def message_constants(pair, stem="GPSDRaw"):
    text = generated().get(f"{gen.PACKAGE}/msg/{stem}{pair[0]}v{pair[1]}.msg") or ""
    return {line.split()[1] for line in text.splitlines() if line.startswith("uint64 ")}


ALL_PAIRS = sorted(gen.REFERENCE_REVS)


@unittest.skipUnless(_HAVE_REPO, f"no GPSd clone at {GPSD_REPO}")
class ManualExpectations(unittest.TestCase):
    """Hand-written per-version expectations, with the reason for each.

    Every entry states a specific GPSd change. If one of these fails, the
    message is no longer a faithful picture of that API version.
    """

    # What used to be a sub-message is now a prefix on the flat root. The
    # assertions below still name the struct they care about; this maps it to
    # where those fields live.
    STEM_TO_PREFIX = {
        "GPSDRaw": "",
        "GPSDFix": "fix_",
        "GPSDSatellite": "skyview_",
        "GPSDDop": "dop_",
        "GPSDDevconfig": "dev_",
        "GPSDPolicy": "policy_",
        "GPSDGst": "gst_",
        "GPSDAttitude": "attitude_",
        "GPSDLog": "log_",
        "GPSDTimedelta": "toff_",
        "GPSDFixsource": "source_",
        "GPSDRawDevices": "devices_",
    }

    def flat(self, pair, stem):
        return fields_under(pair, self.STEM_TO_PREFIX[stem])

    def assertHas(self, pair, stem, field, why):
        fields = self.flat(pair, stem)
        self.assertTrue(fields, f"{stem} fields missing entirely for API "
                                f"{pair[0]}.{pair[1]} ({why})")
        # `covers` rather than equality: a member that was itself a struct is
        # flattened further, so gps_fix_t::ecef shows up as ecef_x, ecef_y...
        self.assertTrue(
            any(covers(field, f) for f in fields),
            f"API {pair[0]}.{pair[1]}: {stem} should have {field!r} -- {why}")

    def assertLacks(self, pair, stem, field, why):
        fields = self.flat(pair, stem)
        if not fields:
            return
        self.assertFalse(
            any(covers(field, f) for f in fields),
            f"API {pair[0]}.{pair[1]}: {stem} should NOT have "
            f"{field!r} -- {why}")

    def test_status_lives_in_gps_data_t_only_on_api_9(self):
        # The defining API 9 -> 10 change: gps_data_t.status moved into
        # gps_fix_t "for better fix merging". It is why GpsdParserV9 exists.
        for pair in ALL_PAIRS:
            if pair[0] == 9:
                self.assertHas(pair, "GPSDRaw", "status",
                               "API 9 keeps the fix status in gps_data_t")
                self.assertLacks(pair, "GPSDFix", "status",
                                 "status does not reach gps_fix_t until API 10")
            else:
                self.assertLacks(pair, "GPSDRaw", "status",
                                 "status left gps_data_t at API 10")
                self.assertHas(pair, "GPSDFix", "status",
                               "status lives in gps_fix_t from API 10 on")

    def test_leap_seconds_arrives_at_9_1(self):
        # The entire content of the 9.0 -> 9.1 bump.
        self.assertLacks((9, 0), "GPSDRaw", "leap_seconds", "added at API 9.1")
        for pair in ALL_PAIRS:
            if pair >= (9, 1):
                self.assertHas(pair, "GPSDRaw", "leap_seconds",
                               "present from API 9.1 on")

    def test_baseline_fields_only_exist_from_13_0(self):
        # API 13 added struct baseline_t and gps_fix_t::base. Flattened, that
        # is fix_base_*; before API 13 no such field may exist at all.
        for pair in ALL_PAIRS:
            if pair >= (13, 0):
                self.assertHas(pair, "GPSDFix", "base",
                               "gps_fix_t::base added at API 13")
            else:
                self.assertLacks(pair, "GPSDFix", "base",
                                 "gps_fix_t::base added at API 13")

    def test_api_14_gps_fix_t_additions(self):
        # These six landed *within* API 14.0 (after 3.24 shipped), which is the
        # whole reason member detection exists. The message is generated from 3.26.1, so it
        # must carry them at 14.0 and must not at 13.0.
        late_14 = ("jam", "temp", "wtemp", "ant_stat", "clockbias", "clockdrift")
        for field in late_14:
            self.assertHas((14, 0), "GPSDFix", field,
                           "added mid-pair within API 14.0 (GPSd 3.25/3.26)")
            self.assertLacks((13, 0), "GPSDFix", field, "not present at API 13")

    def test_api_16_gps_fix_t_additions(self):
        # Listed in the gps.h "15" changelog stanza, but API 15 never existed --
        # a build reports these as API 16 (see plan 1.2).
        for field in ("ant_power", "err_ellipse_orient", "err_ellipse_major",
                      "err_ellipse_minor"):
            for pair in ((16, 0), (16, 1)):
                self.assertHas(pair, "GPSDFix", field,
                               "arrives with API 16 (the never-released '15' stanza)")
            self.assertLacks((14, 0), "GPSDFix", field, "not present at API 14")

    def test_api_14_satellite_additions(self):
        # "Add prRes, prRate, pr, and qualityInd to satellite_t" (API 14).
        for field in ("pr_res", "pr_rate", "pr", "quality_ind"):
            self.assertHas((14, 0), "GPSDSatellite", field, "added to satellite_t at API 14")
            self.assertLacks((12, 0), "GPSDSatellite", field, "not present at API 12")

    def test_fields_present_in_every_version(self):
        # The core of a fix. If any of these ever goes missing the generator is
        # broken in a way no other test would necessarily catch.
        for pair in ALL_PAIRS:
            for field in ("latitude", "longitude", "altitude", "alt_hae", "alt_msl",
                          "track", "speed", "climb", "mode", "ecef", "ned"):
                self.assertHas(pair, "GPSDFix", field, "core gps_fix_t member")
            for field in ("set", "online", "fix", "dop", "skyview",
                          "skyview_time", "satellites_used", "satellites_visible"):
                self.assertHas(pair, "GPSDRaw", field, "core fix member")
            for field in ("prn", "elevation", "azimuth", "ss", "used", "gnssid"):
                self.assertHas(pair, "GPSDSatellite", field, "core satellite_t member")
            for field in ("xdop", "ydop", "pdop", "hdop", "vdop", "tdop", "gdop"):
                self.assertHas(pair, "GPSDDop", field, "dop_t is stable across the range")

    # --- Sensor and device members ----------------------------------------------------------

    def test_source_and_watch_arrive_at_api_14(self):
        # "Add fixsource_t, watch_t, set_pending to gps_data_t" (API 14).
        for pair in ALL_PAIRS:
            if pair >= (14, 0):
                self.assertHas(pair, "GPSDRaw", "source", "fixsource_t added at API 14")
                self.assertHas(pair, "GPSDRaw", "watch", "watch_t added at API 14")
            else:
                self.assertLacks(pair, "GPSDRaw", "source", "not present before API 14")
                self.assertLacks(pair, "GPSDRaw", "watch", "not present before API 14")

    def test_imu_arrives_at_api_12(self):
        # "add imu[], and matching IMU_SET flag" (API 12). attitude_t::msg
        # arrived with it, which is what the parser uses to find the count.
        for pair in ALL_PAIRS:
            if pair >= (12, 0):
                self.assertHas(pair, "GPSDRaw", "imu", "imu[] added at API 12")
                self.assertHas(pair, "GPSDAttitude", "msg",
                               "attitude_t::msg terminates imu[]; added at API 12")
            else:
                self.assertLacks(pair, "GPSDRaw", "imu", "imu[] added at API 12")

    def test_devices_present_in_every_version(self):
        # Unlike source/watch/imu, gps_data_t has carried devices since API 9,
        # which is why the parser fills it without a version guard.
        for pair in ALL_PAIRS:
            self.assertHas(pair, "GPSDRaw", "devices", "present since API 9")
            self.assertHas(pair, "GPSDRawDevices", "ndevices", "the list's count")
            self.assertHas(pair, "GPSDRawDevices", "list", "the device array")

    def test_fixsource_publishes_spec_but_no_pointers(self):
        # server/server_ip/port/device are const char* aimed at caller memory
        # that dangles once gpsd_client's start() returns; spec is a real array
        # carrying the same information.
        for pair in ALL_PAIRS:
            if pair < (14, 0):
                continue
            self.assertHas(pair, "GPSDFixsource", "spec", "a real char array")
            for pointer in ("server", "server_ip", "port", "device"):
                self.assertLacks(pair, "GPSDFixsource", pointer,
                                 "pointer into caller memory, never published")

    def test_sensor_scope_core_members(self):
        for pair in ALL_PAIRS:
            for field in ("dev", "policy", "gst", "attitude", "toff", "pps",
                          "q_err", "q_err_time", "devices"):
                if field in ("gst",) and pair < (10, 0):
                    continue
                self.assertHas(pair, "GPSDRaw", field, "sensor-scope member")

    def test_header_is_first_field_everywhere(self):
        # The spec: every GPSDRaw carries a ROS header, following GPSFix.msg.
        for pair in ALL_PAIRS:
            self.assertEqual(message_fields(pair, "GPSDRaw")[0], "header")

    def test_mask_constants_track_their_version(self):
        # Bit assignments are append-only (plan 1.5), so newer pairs define
        # strictly more constants and never renumber.
        for pair in ALL_PAIRS:
            constants = message_constants(pair)
            self.assertIn("SET_LATLON", constants)
            self.assertIn("SET_UNION", constants)
            # AIS is never published, but the constant must survive, so a
            # consumer can detect an AIS report the message omits.
            self.assertIn("SET_AIS", constants)
            self.assertEqual("SET_SPARTN" in constants, pair[0] == 16,
                             f"API {pair}: SPARTN_SET arrives with API 16")
            self.assertEqual("SET_EOF" in constants, pair >= (14, 0),
                             f"API {pair}: EOF_SET arrives with API 14")
            self.assertEqual("SET_IMU" in constants, pair >= (12, 0),
                             f"API {pair}: IMU_SET arrives with API 12")

    def test_no_constant_collides_with_a_gps_h_macro(self):
        # rosidl emits constants as static constexpr members, so any name
        # gps.h defines as a macro is destroyed by the preprocessor.
        #
        # This checks the whole macro namespace rather than the <NAME>_SET
        # pattern. The earlier pattern-based version passed while emitting
        # SET_HIGH_BIT -- which is gps.h's own macro, and broke the build the
        # moment gpsd_client included a generated message. A carve-out for the
        # one name that was actually broken is exactly the wrong shape of test.
        for pair in ALL_PAIRS:
            macros = gen.macro_names(read_gps_h(gen.REFERENCE_REVS[pair]))
            for name in message_constants(pair):
                self.assertNotIn(
                    name, macros,
                    f"API {pair[0]}.{pair[1]}: constant {name} is also a gps.h "
                    f"macro; it cannot survive being parsed after gps.h")

    def test_set_constants_never_use_the_gps_h_spelling(self):
        for pair in ALL_PAIRS:
            for name in message_constants(pair):
                self.assertFalse(name.endswith("_SET"),
                                 f"API {pair}: {name} uses the gps.h spelling")

    def test_no_message_for_api_15(self):
        self.assertFalse([p for p in ALL_PAIRS if p[0] == 15])
        for path in generated():
            self.assertNotRegex(path, r"15v\d", "API 15 never existed")

    def test_ais_is_absent_everywhere(self):
        # Nothing named ais may appear in any generated message.
        #
        # Note this passes in the fix scope for a weaker reason than it looks:
        # `ais` is in no scope list, so the scope filter drops it before the
        # exclusion list is consulted. test_exclusion_list_wins_over_scope
        # below is what actually exercises the exclusion.
        for path, text in generated().items():
            if path.endswith(".msg"):
                for field in message_fields_of(text):
                    self.assertNotIn("ais", field.lower().split("_"),
                                     f"{path}: AIS is out of scope")

    def test_exclusion_list_wins_over_scope(self):
        # The real test of EXCLUDED_MEMBERS: hand it a scope that does ask for
        # the excluded members and confirm they still never reach a message.
        # Without this, deleting an entry from EXCLUDED_MEMBERS would go
        # unnoticed until the scope that needs it lands.
        pair = (16, 1)
        src = gen.strip_comments(read_gps_h(gen.REFERENCE_REVS[pair]))
        greedy = tuple(gen.FIX_MEMBERS) + gen.EXCLUDED_MEMBERS
        model = gen.build_model(pair, src, greedy)
        fields = [f.name for f in model.messages[f"GPSDRaw{pair[0]}v{pair[1]}"]]
        for excluded in gen.EXCLUDED_MEMBERS:
            self.assertNotIn(gen.snake_case(excluded), fields,
                             f"{excluded} was requested by the scope but must "
                             f"still be excluded")
        # Compared as sets: the skip list follows gps.h declaration order, not
        # the order of EXCLUDED_MEMBERS.
        self.assertEqual(set(model.skipped["gps_data_t"]), set(gen.EXCLUDED_MEMBERS),
                         "every excluded member should be reported as skipped")

    def test_excluded_members_are_absent_everywhere(self):
        for pair in ALL_PAIRS:
            fields = message_fields(pair, "GPSDRaw")
            for excluded in ("gps_fd", "update_fd", "privdata", "set_pending"):
                self.assertNotIn(gen.snake_case(excluded), fields,
                                 f"API {pair}: {excluded} is process-local, never published")


def message_typed_fields(pair, stem):
    """{field name: ROS type} for a generated message, or None."""
    text = generated().get(f"{gen.PACKAGE}/msg/{stem}{pair[0]}v{pair[1]}.msg")
    if text is None:
        return None
    out = {}
    for line in text.splitlines():
        if line and not line.startswith("#") and "=" not in line:
            ros_type, name = line.split()
            out[name] = ros_type
    return out


def fields_under(pair, prefix):
    """Flat root fields under a path prefix, with the prefix stripped.

    Messages are flat now: what used to be GPSDFix's fields are the root's
    `fix_*` fields. Tests still ask "does every member of gps_fix_t reach the
    message", which is the assertion that matters; only where to look changed.
    """
    fields = message_fields(pair, "GPSDRaw")
    if fields is None:
        return None
    return [f[len(prefix):] for f in fields if f.startswith(prefix)]


def covers(member_snake, field):
    """Does `field` come from `member_snake`?

    Exact for a leaf (`mode` -> `mode`), prefixed for a struct that was itself
    flattened (`ecef` -> `ecef_x`). Matching on the member rather than
    splitting the field is deliberate: snake_case names contain underscores,
    so `alt_hae` cannot be split back into segments unambiguously.
    """
    return field == member_snake or field.startswith(member_snake + "_")


def message_fields_of(text):
    return [line.split()[-1] for line in text.splitlines()
            if line and not line.startswith("#") and "=" not in line]


# ---------------------------------------------------------------------------
# Independent completeness cross-check
# ---------------------------------------------------------------------------

def read_gps_h(rev):
    for path in ("include/gps.h", "gps.h"):
        proc = subprocess.run(["git", "-C", GPSD_REPO, "show", f"{rev}:{path}"],
                              capture_output=True, text=True)
        if proc.returncode == 0:
            return proc.stdout
    raise AssertionError(f"cannot read gps.h at {rev}")


def scan_struct_members(src, tag):
    """Re-extract member names from a struct body, independently of the generator.

    Deliberately a different, simpler implementation than split_members(): if
    this shared code with the generator, a dropped member would be dropped from
    the expectations too and the test would pass while the message was wrong.

    Conservative by design. It skips lines it is not confident about (nested
    braces, function pointers, preprocessor lines), because under-reporting
    weakens the test while over-reporting would fail it spuriously. It still
    catches the failure that matters: a member present in gps.h and missing from
    the message.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    src = re.sub(r"//[^\n]*", "", src)
    match = re.search(rf"^struct\s+{tag}\s*\{{", src, re.M)
    if not match:
        return set()
    index, depth = match.end(), 1
    while depth:
        if src[index] == "{":
            depth += 1
        elif src[index] == "}":
            depth -= 1
        index += 1
    body = src[match.end():index - 1]
    body = re.sub(r"\\\s*\n", " ", body)
    # Drop preprocessor lines *before* splitting on ';'. gps.h interleaves
    # #defines with members (MODE_* between `int mode;` and `double ept;`), so
    # a semicolon-split chunk can begin with a #define and end with a real
    # member -- discarding the whole chunk would silently lose that member and
    # make this checker weaker than the generator it is meant to audit.
    body = "\n".join(line for line in body.split("\n")
                     if not line.strip().startswith("#"))

    names, depth = set(), 0
    for statement in body.split(";"):
        depth += statement.count("{") - statement.count("}")
        statement = " ".join(statement.split())
        if not statement or statement.startswith("#") or "(" in statement:
            continue
        if "{" in statement or "}" in statement:
            # Closing an inline struct: `} ecef` names the member.
            tail = re.search(r"\}\s*([A-Za-z_]\w*)\s*$", statement)
            if tail:
                names.add(tail.group(1))
            continue
        if depth > 0:
            continue          # inside an inline struct/union body
        parts = statement.split(",")
        head = re.match(r"^(.*?)\b([A-Za-z_]\w*)\s*(\[[^\]]*\])?$", parts[0])
        # Pointer members are never published -- fixsource_t's server/port/
        # device are const char* into caller memory that dangles once
        # gpsd_client's start() returns. Mirrored here so the completeness
        # check does not demand a field the generator deliberately omits.
        if head and len(parts[0].split()) >= 2 and not head.group(1).rstrip().endswith("*"):
            names.add(head.group(2))
        for extra in parts[1:]:
            extra = extra.strip()
            if extra.startswith("*"):
                continue
            simple = re.match(r"^([A-Za-z_]\w*)\s*(\[[^\]]*\])?$", extra)
            if simple:
                names.add(simple.group(1))
    return names


@unittest.skipUnless(_HAVE_REPO, f"no GPSd clone at {GPSD_REPO}")
class Completeness(unittest.TestCase):
    """Every member gps.h declares must reach the message, or be excluded."""

    # Every struct reachable from a published gps_data_t member. Structs that
    # do not exist in an older gps.h are handled per-pair below rather than
    # excluded, so "message absent" and "struct absent" must agree.
    # struct tag -> the flat field prefix its members land under. Where a
    # struct appears twice (attitude_t as both `attitude` and `imu`,
    # timedelta_t as `toff` and `pps`), one representative is enough: the
    # generator emits them from the same code path.
    STRUCT_TO_PREFIX = {
        # Core fix members
        "gps_fix_t": "fix_",
        "satellite_t": "skyview_",
        "dop_t": "dop_",
        # Sensor and device members
        "devconfig_t": "dev_",
        "gps_policy_t": "policy_",
        "gst_t": "gst_",
        "attitude_t": "attitude_",
        "gps_log_t": "log_",
        "timedelta_t": "toff_",
        "fixsource_t": "source_",
    }

    def test_every_struct_member_reaches_its_message(self):
        for pair in ALL_PAIRS:
            src = read_gps_h(gen.REFERENCE_REVS[pair])
            for tag, prefix in self.STRUCT_TO_PREFIX.items():
                fields = fields_under(pair, prefix)
                declared = scan_struct_members(src, tag)
                if not fields:
                    # No fields is only acceptable when gps.h has no such
                    # struct at this revision. Otherwise a struct was dropped.
                    self.assertEqual(
                        declared, set(),
                        f"API {pair[0]}.{pair[1]}: {tag} exists in gps.h but "
                        f"no {prefix}* fields were generated")
                    continue
                for member in sorted(declared):
                    if member in gen.EXCLUDED_MEMBERS:
                        continue
                    snake = gen.snake_case(member)
                    self.assertTrue(
                        any(covers(snake, f) for f in fields),
                        f"API {pair[0]}.{pair[1]} ({gen.REFERENCE_REVS[pair]}): "
                        f"{tag}.{member} exists in gps.h but no {prefix}{snake} "
                        f"field")

    def test_fix_scope_members_of_gps_data_t_reach_the_root_message(self):
        for pair in ALL_PAIRS:
            src = read_gps_h(gen.REFERENCE_REVS[pair])
            declared = scan_struct_members(src, "gps_data_t")
            fields = message_fields(pair, "GPSDRaw")
            for member in sorted(declared):
                if member in gen.EXCLUDED_MEMBERS or member not in gen.FIX_MEMBERS:
                    continue
                snake = gen.snake_case(member)
                self.assertTrue(
                    any(covers(snake, f) for f in fields),
                    f"API {pair[0]}.{pair[1]}: gps_data_t.{member} is in the "
                    f"fix scope but has no field in GPSDRaw")

    def test_scanner_disagrees_with_nothing_it_should_agree_with(self):
        # Guard on the guard: if the independent scanner silently returned
        # (almost) nothing, every completeness assertion above would pass
        # vacuously. gps_fix_t is 28+ members in every supported version.
        for pair in ALL_PAIRS:
            src = read_gps_h(gen.REFERENCE_REVS[pair])
            self.assertGreater(len(scan_struct_members(src, "gps_fix_t")), 25,
                               f"API {pair}: scanner found too few members to be trusted")
            self.assertGreater(len(scan_struct_members(src, "satellite_t")), 5)
            self.assertEqual(len(scan_struct_members(src, "dop_t")), 7)

    def test_message_has_no_field_without_a_gps_h_member(self):
        # The other direction: a field with nothing behind it would be a
        # fabricated value in a message that claims to be raw.
        for pair in ALL_PAIRS:
            src = read_gps_h(gen.REFERENCE_REVS[pair])
            root_members = {gen.snake_case(m)
                            for m in scan_struct_members(src, "gps_data_t")}
            for tag, prefix in self.STRUCT_TO_PREFIX.items():
                fields = fields_under(pair, prefix)
                if not fields:
                    continue
                declared = {gen.snake_case(m) for m in scan_struct_members(src, tag)}
                for field in fields:
                    # A prefix can also match a gps_data_t member of its own:
                    # `skyview_time` is gps_data_t::skyview_time, not a
                    # satellite_t field that happens to sit under skyview_.
                    if prefix + field in root_members:
                        continue
                    self.assertTrue(
                        any(covers(m, field) for m in declared),
                        f"API {pair[0]}.{pair[1]}: {prefix}{field} has no "
                        f"corresponding member in {tag}")


@unittest.skipUnless(_HAVE_REPO, f"no GPSd clone at {GPSD_REPO}")
class FieldTypes(unittest.TestCase):
    """Field *types*, not just names.

    Membership tests alone let a type regression through: flipping double to
    float32 in the mapping table silently halves the precision of every field
    in every message while every name assertion still passes. Mutation-testing
    this suite is what surfaced that, so the types are pinned explicitly.
    """

    # Types that must hold for these fields in every supported API pair.
    STABLE = {
        "GPSDRaw": {
            "header": "std_msgs/Header",
            "set": "uint64",                       # gps_mask_t, verbatim
            "online": "builtin_interfaces/Time",   # timespec_t
            "skyview_time": "builtin_interfaces/Time",
            "satellites_used": "int32",
            "satellites_visible": "int32",
        },
        "GPSDFix": {
            "time": "builtin_interfaces/Time",
            "mode": "int32",
            "latitude": "float64",
            "longitude": "float64",
            "altitude": "float64",
            "alt_hae": "float64",
            "alt_msl": "float64",
            "track": "float64",
            "speed": "float64",
            "climb": "float64",
            "eph": "float64",
            "epv": "float64",
            "ept": "float64",
        },
        "GPSDSatellite": {
            "prn": "int16",       # int16_t
            "elevation": "float64",
            "azimuth": "float64",
            "ss": "float64",
            "used": "bool",
            "gnssid": "uint8",    # uint8_t
            "freqid": "int8",     # int8_t
        },
        "GPSDDop": {name: "float64" for name in
                    ("xdop", "ydop", "pdop", "hdop", "vdop", "tdop", "gdop")},
    }

    # Prefix each STABLE group lands under, and whether it became a parallel
    # array. skyview is the only one here that did.
    STEM_TO_PREFIX = {"GPSDRaw": "", "GPSDFix": "fix_",
                      "GPSDSatellite": "skyview_", "GPSDDop": "dop_"}
    ARRAY_STEMS = {"GPSDSatellite"}

    def test_stable_field_types(self):
        for pair in ALL_PAIRS:
            actual = message_typed_fields(pair, "GPSDRaw")
            self.assertIsNotNone(actual, f"API {pair}: GPSDRaw not generated")
            for stem, expected in self.STABLE.items():
                prefix = self.STEM_TO_PREFIX[stem]
                suffix = "[]" if stem in self.ARRAY_STEMS else ""
                for name, ros_type in expected.items():
                    flat = prefix + name
                    if flat not in actual:
                        continue        # covered by the membership tests
                    self.assertEqual(
                        actual[flat], ros_type + suffix,
                        f"API {pair[0]}.{pair[1]}: {flat} should be "
                        f"{ros_type}{suffix}, got {actual[flat]}")

    def test_no_project_defined_sub_messages_remain(self):
        """Flattening's defining property: only standard ROS types nest.

        Replaces the old check that sub-message references carried the version
        suffix. There are no sub-messages to get wrong now -- which is the
        stronger guarantee, since a GPSDRaw16v1 pointing at a GPSDFix14v0 was
        the failure that check existed to catch.
        """
        for pair in ALL_PAIRS:
            for name, ros_type in message_typed_fields(pair, "GPSDRaw").items():
                base = ros_type[:-2] if ros_type.endswith("[]") else ros_type
                self.assertFalse(
                    base.startswith(gen.MESSAGE_PREFIX),
                    f"API {pair[0]}.{pair[1]}: {name} is {ros_type}; the root "
                    f"message must not reference another generated message")
                if "/" in base:
                    self.assertIn(base.split("/")[0],
                                  ("std_msgs", "builtin_interfaces"),
                                  f"{name}: unexpected package for {ros_type}")

    def test_skyview_is_an_unbounded_array(self):
        # MAXCHANNELS is 140 or 184 depending on the rev and is not a
        # function of the API pair, so it must not appear in any message type.
        for pair in ALL_PAIRS:
            self.assertTrue(
                message_typed_fields(pair, "GPSDRaw")["skyview_prn"].endswith("[]"))
        for path, text in generated().items():
            if path.endswith(".msg"):
                self.assertNotRegex(text, r"\[\s*\d+\s*\]",
                                    f"{path}: fixed-size array leaks a C extent")

    def test_char_array_becomes_string(self):
        for pair in ALL_PAIRS:
            raw = message_typed_fields(pair, "GPSDRaw")
            if "fix_datum" in raw:          # char datum[40]
                self.assertEqual(raw["fix_datum"], "string")

    def test_types_agree_with_the_c_declarations(self):
        """Independent cross-check: re-read each C type and map it separately."""
        expected_for_c = {
            "double": "float64", "float": "float32", "bool": "bool",
            "int": "int32", "unsigned": "uint32", "unsigned int": "uint32",
            "long": "int64", "unsigned long": "uint64",
            "int8_t": "int8", "uint8_t": "uint8",
            "int16_t": "int16", "uint16_t": "uint16",
            "int32_t": "int32", "uint32_t": "uint32",
            "int64_t": "int64", "uint64_t": "uint64",
            "timespec_t": "builtin_interfaces/Time",
            "gps_mask_t": "uint64", "gnssid_t": "uint8", "time_t": "int64",
        }
        # Groups that flattened into parallel arrays carry a [] the C
        # declaration does not; everything else keeps its scalar type.
        array_prefixes = ("skyview_", "imu_", "devices_list_", "raw_meas_")
        checked = 0
        for pair in ALL_PAIRS:
            src = read_gps_h(gen.REFERENCE_REVS[pair])
            actual = message_typed_fields(pair, "GPSDRaw")
            for tag, prefix in Completeness.STRUCT_TO_PREFIX.items():
                for member, ctype in scan_member_types(src, tag).items():
                    if member in gen.EXCLUDED_MEMBERS:
                        continue
                    want = expected_for_c.get(ctype)
                    name = prefix + gen.snake_case(member)
                    if want is None or name not in actual:
                        continue
                    if name.startswith(array_prefixes):
                        want += "[]"
                    self.assertEqual(
                        actual[name], want,
                        f"API {pair[0]}.{pair[1]}: {tag}.{member} is {ctype!r} "
                        f"so {name} should be {want}, got {actual[name]}")
                    checked += 1
        # Guard on the guard: a scanner that returned nothing would pass.
        self.assertGreater(checked, 300, "cross-check covered too few fields")


def union_members(src, tag):
    """Names of the members inside a struct's anonymous union.

    Independent of the generator, like scan_struct_members: sharing that code
    would let a dropped arm be dropped from the expectation too.
    """
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    src = re.sub(r"//[^\n]*", "", src)
    match = re.search(rf"^struct\s+{tag}\s*\{{", src, re.M)
    if not match:
        return set()
    index, depth = match.end(), 1
    while depth:
        index += 1
        if src[index] == "{":
            depth += 1
        elif src[index] == "}":
            depth -= 1
    body = src[match.end():index]
    # The report union is the anonymous `union { ... };` at this level.
    um = re.search(r"\bunion\s*\{", body)
    if not um:
        return set()
    i, d = um.end(), 1
    while d:
        if body[i] == "{":
            d += 1
        elif body[i] == "}":
            d -= 1
        i += 1
    names = set()
    for statement in body[um.end():i - 1].split(";"):
        statement = " ".join(statement.split())
        if not statement or "(" in statement or "{" in statement:
            continue
        m2 = re.match(r"^.*?\b([A-Za-z_]\w*)\s*(\[[^\]]*\])?$", statement)
        if m2 and len(statement.split()) >= 2:
            names.add(m2.group(1))
    return names


def scan_member_types(src, tag):
    """{member: C type} from a struct body, independently of the generator."""
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    src = re.sub(r"//[^\n]*", "", src)
    match = re.search(rf"^struct\s+{tag}\s*\{{", src, re.M)
    if not match:
        return {}
    index, depth = match.end(), 1
    while depth:
        if src[index] == "{":
            depth += 1
        elif src[index] == "}":
            depth -= 1
        index += 1
    body = src[match.end():index - 1]
    body = re.sub(r"\\\s*\n", " ", body)
    body = "\n".join(line for line in body.split("\n")
                      if not line.strip().startswith("#"))

    out, depth = {}, 0
    for statement in body.split(";"):
        depth += statement.count("{") - statement.count("}")
        text = " ".join(statement.split())
        if not text or "(" in text or "{" in text or "}" in text or depth > 0:
            continue
        parts = text.split(",")
        head = re.match(r"^(.*?)\b([A-Za-z_]\w*)\s*(\[[^\]]*\])?$", parts[0].strip())
        if not head or not head.group(1).strip():
            continue
        ctype = head.group(1).strip()
        if head.group(3) is None:          # skip arrays; string/uint8[] rules
            out[head.group(2)] = ctype
        for extra in parts[1:]:
            simple = re.match(r"^([A-Za-z_]\w*)$", extra.strip())
            if simple:
                out[simple.group(1)] = ctype
    return out


_REPO_ROOT = os.path.dirname(_HERE)


@unittest.skipUnless(_HAVE_REPO, f"no GPSd clone at {GPSD_REPO}")
class CheckedInFilesAreUpToDate(unittest.TestCase):
    """The colcon-test equivalent of `generate_raw_msgs.py --check`.

    The generated messages and parser headers are checked in, because rosidl
    needs the .msg files in-tree and reviewers need to see generated C++. That
    only stays trustworthy if regenerating is verified to be a no-op, so the
    drift check runs as an ordinary test rather than only in a bespoke CI step.
    """

    def test_no_drift_between_generator_and_checked_in_files(self):
        stale = []
        for path, contents in sorted(generated().items()):
            full = os.path.join(_REPO_ROOT, path)
            if not os.path.exists(full):
                stale.append(f"missing: {path}")
            elif open(full).read() != contents:
                stale.append(f"differs: {path}")
        self.assertEqual(
            stale, [],
            "checked-in generated files are out of date; run "
            "tools/generate_raw_msgs.py")

    def test_no_orphaned_generated_files(self):
        """Nothing left behind that the generator no longer produces.

        The message package globs its msg/ directory, so a file left over from
        a rename is still built -- which is how a renamed message once produced
        two conflicting definitions of the same type and broke the build. The
        drift check above only notices missing or differing files, never extra
        ones, so this is a separate assertion.
        """
        stale = gen.orphans(generated(), _REPO_ROOT)
        self.assertEqual(stale, [],
                         "stale generated files; rerun tools/generate_raw_msgs.py")

    def test_every_generated_message_is_listed_for_rosidl(self):
        # gps_msgs globs msg/GPSD*.msg, so a message whose name did not match
        # the glob would generate cleanly and then never be built. The prefix
        # is also what keeps the generated messages distinct from the
        # hand-written GPSFix/GPSStatus sharing the directory.
        for path in generated():
            if path.endswith(".msg"):
                self.assertRegex(
                    os.path.basename(path),
                    rf"^{gen.MESSAGE_PREFIX}.*\.msg$",
                    f"{path} would not be picked up by the glob "
                    f"in {gen.PACKAGE}/CMakeLists.txt")

    def test_one_ci_workflow_per_api_pair(self):
        # Each API pair gets its own workflow so a failure names the version.
        # Generated from the same manifest as the messages, so adding a pair
        # cannot leave it untested.
        for pair in ALL_PAIRS:
            path = f".github/workflows/gpsd_api_{pair[0]}v{pair[1]}.yml"
            self.assertIn(path, generated(), f"API {pair} has no CI workflow")
            body = generated()[path]
            self.assertIn(f"name: gpsd API {pair[0]}.{pair[1]}", body)
            self.assertIn(f"msg: 'GPSDRaw{pair[0]}v{pair[1]}'", body)
            # Must call the shared workflow, not carry its own copy of the logic.
            self.assertIn("uses: ./.github/workflows/gpsd_api_shared.yml", body)
            # The revision must be the one the message was generated from.
            rev = gen.REFERENCE_REVS[pair].replace("release-", "")
            self.assertIn(f"gpsd: '{rev}'", body)

    def test_message_names_survive_rosidls_normalisation(self):
        """No consecutive capitals after the fixed prefix.

        rosidl derives the C struct name by normalising a run of capitals
        (NED -> Ned) but writes the name as authored into the *referencing*
        message's header. A name like GPSDFixNED16v1 therefore yields
        two spellings that disagree, and the generated C fails to build with
        "unknown type name". Caught the hard way; guarded here.
        """
        for path in generated():
            if not path.endswith(".msg"):
                continue
            name = os.path.basename(path)[:-len(".msg")]
            tail = name[len(gen.MESSAGE_PREFIX):]
            self.assertNotRegex(
                tail, r"[A-Z]{2}",
                f"{name}: consecutive capitals in {tail!r} will not survive "
                f"rosidl's name normalisation")

    def test_no_workflow_for_an_api_pair_that_does_not_exist(self):
        workflows = [p for p in generated()
                     if p.startswith(".github/workflows/gpsd_api_")]
        self.assertEqual(len(workflows), len(ALL_PAIRS))
        for path in workflows:
            self.assertNotRegex(path, r"gpsd_api_15v", "API 15 never existed")


@unittest.skipUnless(_HAVE_REPO, f"no GPSd clone at {GPSD_REPO}")
class GeneratedParserCode(unittest.TestCase):
    """Cheap guards on the emitted C++.

    The message tests cannot see these: changing how a timespec_t is filled,
    for instance, leaves every .msg byte-identical and only corrupts the
    generated fill code. Compiling and running that code is phase 3/5 work;
    until then these text assertions catch the obvious regressions.
    """

    def parser_source(self, pair):
        return generated()[
            "gpsd_client/include/gpsd_client/parsers/generated/"
            f"gpsd_raw_fill_{pair[0]}v{pair[1]}.hpp"]

    def test_guarded_on_the_exact_api_pair(self):
        for pair in ALL_PAIRS:
            source = self.parser_source(pair)
            self.assertIn(
                f"#if GPSD_RAW_FILL_MAJOR == {pair[0]} && "
                f"GPSD_RAW_FILL_MINOR == {pair[1]}", source,
                "a parser must not compile against the wrong API pair")
            # The guard is overridable so the ladder can point a
            # newer-than-tested libgps at the newest parser, but it must still
            # default to this build's own version when included standalone.
            self.assertIn("#define GPSD_RAW_FILL_MAJOR GPSD_API_MAJOR_VERSION",
                          source)
            self.assertIn("#define GPSD_RAW_FILL_MINOR GPSD_API_MINOR_VERSION",
                          source)

    def test_selection_ladder_covers_every_pair_and_both_edges(self):
        ladder = generated()["gpsd_client/include/gpsd_client/gpsd_raw_message.hpp"]
        for pair in ALL_PAIRS:
            self.assertIn(f"#define GPSD_RAW_FILL_MAJOR {pair[0]}", ladder)
            self.assertIn(
                f"#include <gpsd_client/parsers/generated/"
                f"gpsd_raw_fill_{pair[0]}v{pair[1]}.hpp>", ladder)
            self.assertIn(
                f"using GpsdRawMsg = {gen.PACKAGE}::msg::GPSDRaw{pair[0]}v{pair[1]};",
                ladder)
        # Matches the policy in gpsd_parser_factory.cpp: hard error below the
        # minimum, warn and fall back to newest above the maximum.
        self.assertIn("#error", ladder)
        self.assertIn("#warning", ladder)
        newest = ALL_PAIRS[-1]
        self.assertIn(f"falling back to the API {newest[0]}.{newest[1]}", ladder)

    def test_timespec_members_fill_sec_and_nanosec(self):
        # gps.h timespec_t -> builtin_interfaces/Time is a two-field split; a
        # plain assignment would not compile, or worse would silently drop the
        # nanoseconds if the types ever became assignable.
        for pair in ALL_PAIRS:
            source = self.parser_source(pair)
            for field in ("online", "skyview_time"):
                self.assertIn(f"out.{field}.sec = ", source)
                self.assertIn(f"out.{field}.nanosec = ", source)

    def test_every_assignment_is_member_guarded(self):
        # An unguarded assignment is exactly the bug that breaks a build
        # against an older libgps reporting the same API pair.
        for pair in ALL_PAIRS:
            for line in self.parser_source(pair).splitlines():
                stripped = line.strip()
                if stripped.startswith("out.") and stripped.endswith(";"):
                    self.assertTrue(
                        stripped.startswith("out."),
                        f"API {pair}: unguarded assignment {stripped!r}")
            # Structurally: as many if constexpr guards as assignment blocks.
            source = self.parser_source(pair)
            self.assertGreater(source.count("if constexpr (has_"), 20,
                               f"API {pair}: too few member guards to be real")

    def test_every_union_arm_is_mask_gated(self):
        """Reading an arm the mask does not name is a read of an inactive
        union member -- undefined behaviour, not merely a stale value.

        Scans gps.h for gps_data_t's union block independently of the
        generator, then requires the fill to guard each arm on `in.set`. This
        exists because the arms are plain scalars on the flat message: nothing
        in the message shape hints that they need a guard, and `attitude` and
        `gst` are union members on API 9-11 but ordinary members from API 12,
        so the answer changes with the version.
        """
        for pair in ALL_PAIRS:
            src = read_gps_h(gen.REFERENCE_REVS[pair])
            arms = union_members(src, "gps_data_t")
            self.assertGreater(len(arms), 3,
                               f"API {pair}: found too few union arms to trust "
                               f"the scanner")
            source = self.parser_source(pair)
            fields = message_fields(pair, "GPSDRaw")
            checked = 0
            for arm in sorted(arms):
                if arm in gen.EXCLUDED_MEMBERS:
                    continue
                # An arm outside every publishing scope (navdata, ais) reaches
                # no field, so nothing reads it and nothing needs a gate. Key
                # on what was actually published rather than on a second list
                # that could drift from the scopes.
                snake = gen.snake_case(arm)
                if not any(covers(snake, f) for f in fields):
                    continue
                checked += 1
                bit = gen.REPORT_UNION_BITS.get(arm)
                self.assertIsNotNone(
                    bit, f"API {pair[0]}.{pair[1]}: gps_data_t's union has "
                         f"{arm!r} with no entry in REPORT_UNION_BITS, so the "
                         f"fill cannot gate it")
                self.assertIn(
                    f"in.set & {bit}", source,
                    f"API {pair[0]}.{pair[1]}: union arm {arm!r} is copied "
                    f"without checking {bit}")
            self.assertGreater(
                checked, 2,
                f"API {pair}: checked too few published union arms to be real")

    def test_array_groups_are_filled_from_one_loop(self):
        """Parallel arrays in a group must be written together.

        Flattening removed the type-level guarantee that one array of structs
        stays self-consistent, so the generated filler resizes and writes every
        array in a group inside a single loop. A per-array loop would compile
        and pass every membership test while letting the arrays drift apart.

        Also checks the count still comes from the caller: only it knows how
        many elements are valid, and a blind loop over the C array would
        publish MAXCHANNELS entries of garbage.
        """
        for pair in ALL_PAIRS:
            source = self.parser_source(pair)
            self.assertIn("inline void fill_skyview(", source)
            self.assertIn("const std::vector<std::size_t>& idx", source)
            body = source[source.index("inline void fill_skyview("):]
            body = body[:body.index("\ninline void ", 1)] if "\ninline void " \
                in body[1:] else body
            self.assertEqual(
                1, body.count("for (std::size_t i = 0"),
                f"API {pair}: fill_skyview must write its arrays from one loop")
            self.assertGreater(body.count(".resize(count)"), 5,
                               f"API {pair}: too few arrays resized together")

    def test_includes_match_rosidl_header_names(self):
        for pair in ALL_PAIRS:
            source = self.parser_source(pair)
            suffix = f"{pair[0]}v{pair[1]}"
            self.assertIn(f"#include <{gen.PACKAGE}/msg/gpsd_raw{suffix}.hpp>", source)
            # One message now, so this is the only include to get right. The
            # acronym split is the part that is easy to regress.
            self.assertNotIn("gpsdraw", source)


if __name__ == "__main__":
    unittest.main()
