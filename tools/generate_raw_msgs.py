#!/usr/bin/env python3
"""Generate the GPSDRaw<MAJOR>v<MINOR> messages and their parsers from gps.h.

Ground truth is GPSd's ``include/gps.h`` at a pinned commit per API pair (see
REFERENCE_REVS). Nothing here reads the build host's installed libgps: the
messages live in ``gps_extended_msgs``, a pure interface package that must
never gain a libgps dependency. Regenerating is a
deliberate, reviewed act, and CI runs ``--check`` so the checked-in output
cannot drift from this script.

The rules that constrain this script most: the message package never gains a
libgps dependency, messages are generated rather than hand-written, every
sub-message carries the version suffix, union arms are selected by the ``set``
mask, the mask constants are renamed ``SET_<NAME>``, AIS is out of scope, and
absent members are detected in C++ rather than assumed.


Why a commit and not an API version
-----------------------------------

An API pair names a *range* of header states, not one state. GPSd bumps the
version when a change begins and then keeps adding under the same number until
the next bump. Both of these are real:

  * API 14.0 covers GPSd 3.24 through 3.26.1, and ``gps_fix_t`` gains
    ant_stat, clockbias, clockdrift, jam, temp and wtemp across that range.
  * API 16.1 covers 3.27.5 (MAXCHANNELS 184) and master (230).

So a message generated from a pair's last rev is a *superset* of what some
libgps reporting that same pair provides, and generated parser code must guard
every field with the C++ detection idiom rather than assume presence.

Do not trust the changelog comment block at the top of gps.h for values. It
claims MAXCHANNELS went to 185 and then 230; no release ever shipped either.
Read the ``#define``.


Type mapping
------------

Applied uniformly so generated output can be checked against a stated rule.

  GPSd C type              ROS 2 field                      Note
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
  struct X                 GPSD<X><MAJOR>v<MINOR>
  struct X arr[N]          GPSD<X><MAJOR>v<MINOR>[]         unbounded
  C union                  mask + all arms, one populated
  NaN sentinel             float64 NaN preserved            GPSd uses NaN for
                                                            "unknown"; never zero

Fixed C arrays become unbounded ROS arrays truncated to the valid count
(satellites_visible, devices.ndevices, ...). MAXCHANNELS is deliberately not
encoded in any message type: it is 140 or 184 depending on the rev, and it is
not a function of the API pair, so baking it in would be ambiguous as well as
wasteful.

The ``set`` bitmask constants are emitted as uint64 message constants, renamed
``<NAME>_SET`` -> ``SET_<NAME>``. The rename is forced: rosidl emits
constants as ``static constexpr`` members, gps.h defines the GPSd spellings as
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
# ais       -- out of scope. struct ais_t is a ~two-dozen-arm tagged union of marine
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
    # Published as raw GPSd JSON instead of as typed messages. These three
    # trees were 684 of ~905 generated messages -- 76% of the package -- for
    # data almost no subscriber decodes. gps_read() hands back the JSON line
    # for every report, including classes libgps itself cannot decode, so the
    # JSON topic carries strictly more than the typed path did: SUBFRAME
    # reaches a client there and can never reach gps_data_t::subframe at all.
    #
    # SET_RTCM2, SET_RTCM3 and SET_SUBFRAME stay in the mask constants and
    # `set` is still copied verbatim, so a raw subscriber can still tell GPSd
    # reported one and look at the JSON topic for it. Same contract as AIS.
    "rtcm2",
    "rtcm3",
    "subframe",
)

# API (major, minor) -> the GPSd revision the message is generated from.
#
# Released pairs use the *last* release carrying that pair, so the message is
# the superset of what any libgps reporting the pair provides (see below for how
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
# GPSd 3.27, 3.27.1 and 3.27.2 are out of scope (they shipped
# api_version_major = 0 in SConscript). Nothing is lost: all three are API 16.0,
# represented here by 3.27.3.
# Branches whose *pushes* run the heavy per-API-version jobs. A push to one of
# these builds libgps from source ten times over, so the list is deliberately
# short.
#
# ros2-devel is the permanent entry. Anything else is a feature branch being
# validated before it becomes a pull request -- worth the CI time while the
# branch is live, and worth deleting the moment it merges. Pull requests are
# unaffected: they run on a path filter regardless of branch.
#
# A branch that is not listed still gets the fast checks (gpsd_generator.yml and
# the per-distro workflows run on every push); it only skips the ten GPSd
# source builds.
#
# Adding a branch here is the only way to exercise a *new* workflow before it
# merges: GitHub offers workflow_dispatch only for workflows already on the
# default branch, so the "Run workflow" button does not exist for a file that
# has never been merged. Remove the extra branch once it does merge -- every
# entry costs a full GPSd build per API version on every push.
PUSH_BRANCHES = (
    "ros2-devel",
)

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
# rev was read correctly -- it is never emitted into a message.
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

# Members of gps_data_t grouped by publishing scope. Each scope is a starting
# set; the generator pulls in every struct reachable from it transitively.
FIX_MEMBERS = (
    "set",
    "online",
    "fix",
    "dop",
    "skyview",
    "skyview_time",
    "satellites_used",
    "satellites_visible",
    "leap_seconds",
    "status",           # gps_data_t only on API 9; moved into gps_fix_t at 10
)

SENSOR_MEMBERS = (
    "dev", "devices", "policy", "gst", "attitude", "imu", "log",
    "toff", "pps", "qErr", "qErr_time", "source", "watch",
)

REPORT_MEMBERS = (
    "raw", "osc", "version", "error",
)

# Ordered widest-last: each scope adds to the ones before it, so the order
# here defines what --scope includes. Do not derive it by sorting the names.
SCOPE_ORDER = ("fix", "sensors", "reports")
SCOPES = {
    "fix": FIX_MEMBERS,          # position, satellites, DOP
    "sensors": SENSOR_MEMBERS,   # device config, timing, attitude, IMU
    "reports": REPORT_MEMBERS,   # the report union arms
}

# gps_data_t's report union: which set-mask bit selects each arm.
#
# Mostly <NAME>_SET, but not reliably -- `osc` is selected by OSCILLATOR_SET --
# so the mapping is written out rather than derived from the member name.
#
# There is deliberately no `ais` entry: AIS is not published. AIS_SET
# stays in the mask and in the message constants, so a consumer can still see
# that GPSd reported an AIS message this message does not carry.
REPORT_UNION_BITS = {
    "rtcm2": "RTCM2_SET",
    "rtcm3": "RTCM3_SET",
    "subframe": "SUBFRAME_SET",
    "raw": "RAW_SET",
    "osc": "OSCILLATOR_SET",
    "version": "VERSION_SET",
    "error": "ERROR_SET",
    # attitude and gst sit *inside* the union on API 9-11 and move out of it
    # at API 12. Listed here so they are mask-gated on the versions where they
    # are union arms; flatten_model() only applies a gate when the struct
    # layout says the member really is one, so on API 12+ these are ignored
    # and the fields are copied unconditionally, which is then correct.
    "attitude": "ATTITUDE_SET",
    "gst": "GST_SET",
}

# `sub4` is declared in gps.h and never written by GPSd -- no driver or daemon
# code references it, the same as rtcm2_18..24. It stays empty always.

# Structs published on their own message root rather than inside GPSDRaw.
#
# Empty: nothing needs one. The kept members all belong in the raw report, and
# what used to live here -- RTCM2 and RTCM3 -- now travels as raw JSON.
STANDALONE_ROOTS = ()

# The gps_data_t member names those structs appear under.
STANDALONE_MEMBER_NAMES = ()

# The scope the checked-in files are generated at, and the default for
# --scope. The CLI, the drift check and the tests all read this one value, so
# widening the tree takes a one-line change here plus a regenerate.
CHECKED_IN_SCOPE = "reports"

# The generated messages live in their own package, not in gps_msgs.
#
# gps_msgs is a small, long-released interface package (GPSFix, GPSStatus) that
# the ROS build farm builds for five distros. The generated set is two orders
# of magnitude larger -- the report arms alone are ~90 messages per API pair, and building
# them takes minutes rather than seconds -- so putting them here would impose
# that on every consumer of gps_msgs, released or not. A separate package keeps
# the cost with the feature that incurs it.
#
# Messages keep the GPSD prefix, naming the daemon they mirror, while the
# package name says what it is relative to gps_msgs.
PACKAGE = "gps_extended_msgs"
MESSAGE_PREFIX = "GPSD"


import argparse
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


# --------------------------------------------------------------------------
# Reading gps.h
# --------------------------------------------------------------------------

def read_gps_h(repo: str, rev: str) -> str:
    """Return include/gps.h at `rev`, falling back to the pre-3.22 root path."""
    last = None
    for path in ("include/gps.h", "gps.h"):
        proc = subprocess.run(
            ["git", "-C", repo, "show", f"{rev}:{path}"],
            capture_output=True, text=True)
        if proc.returncode == 0:
            return proc.stdout
        last = proc.stderr.strip()
    raise SystemExit(f"cannot read gps.h at {rev} in {repo}: {last}")


def strip_comments(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def resolve_conditionals(body: str) -> str:
    """Resolve the preprocessor conditionals that appear inside struct bodies.

    Only one exists anywhere in the supported range: `#ifndef USE_QT` around
    gps_data_t::gps_fd, which is excluded anyway. USE_QT is treated as
    undefined because this package never builds against the Qt bindings.

    Anything else raises rather than guessing -- silently emitting both arms of
    a conditional would corrupt the field model.
    """
    # Join backslash continuations first: gps_data_t's UNION_SET is a
    # multi-line #define, and dropping only its first line would leave the
    # remaining arms looking like struct members.
    body = re.sub(r"\\\s*\n", " ", body)

    out, stack = [], []
    for line in body.split("\n"):
        stripped = line.strip()
        if stripped.startswith("#define"):
            continue
        if stripped.startswith("#"):
            if re.match(r"#ifndef\s+USE_QT\b", stripped):
                stack.append(True); continue
            if re.match(r"#ifdef\s+USE_QT\b", stripped):
                stack.append(False); continue
            if stripped.startswith("#else"):
                if not stack:
                    raise SystemExit("unbalanced #else in struct body")
                stack[-1] = not stack[-1]; continue
            if stripped.startswith("#endif"):
                if not stack:
                    raise SystemExit("unbalanced #endif in struct body")
                stack.pop(); continue
            raise SystemExit(
                f"unhandled preprocessor conditional in struct body: {stripped!r}. "
                "Teach resolve_conditionals() about it rather than letting the "
                "field model silently include both arms.")
        if all(stack):
            out.append(line)
    return "\n".join(out)


def find_struct_body(src: str, name: str) -> Optional[str]:
    """Return the brace-balanced body of `struct <name> { ... }`."""
    # Not anchored to line start: GPSd defines several tagged structs *inside*
    # other structs (struct gps_rangesat_t inside rtcm2_t, for one), where they
    # are indented. Requiring `{` after the tag keeps this from matching a mere
    # reference such as `struct gps_fix_t fix;`.
    match = re.search(rf"\bstruct\s+{re.escape(name)}\s*\{{", src)
    if match is None:
        return None
    index, depth = match.end(), 1
    while depth:
        if src[index] == "{":
            depth += 1
        elif src[index] == "}":
            depth -= 1
        index += 1
    return src[match.end():index - 1]


# --------------------------------------------------------------------------
# Field model
# --------------------------------------------------------------------------

@dataclass
class Member:
    name: str
    ctype: str                       # 'double', 'struct gps_fix_t', ...
    array: Optional[str] = None      # array extent as written, or None
    anon_body: Optional[str] = None  # body of an inline anonymous struct
    is_pointer: bool = False         # declared with a '*'
    struct_tag: Optional[str] = None  # tag of an inline `struct X { ... } m;`
    is_union: bool = False           # inline union with a declarator
    union_arm: bool = False          # spliced in from an anonymous union



@dataclass
class StructDef:
    cname: str                       # 'gps_fix_t', or a synthetic name for anon
    members: List[Member] = field(default_factory=list)


def brace_body(text: str, start: int) -> Tuple[str, int]:
    """Return (body, index-after-close) for a block whose '{' is already past."""
    index, depth = start, 1
    while depth:
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
        index += 1
    return text[start:index - 1], index


def split_members(body: str) -> List[Member]:
    """Parse a struct body into members.

    Handles the four shapes gps.h actually uses: plain scalars, multi-declarator
    lines (`double x, y, z;`), arrays, named struct members, and inline
    anonymous structs given a member name (`struct { ... } ecef;`).
    """
    members: List[Member] = []
    index = 0
    while index < len(body):
        # Enums map to a plain integer. gps.h uses both anonymous
        # (`enum {RESERVED, CORRECT, WIDELANE, UNCERTAIN} ambiguity;`) and
        # tagged-by-reference (`enum RTCM3_QUALITY_INDICATOR_TRANSFORMATION
        # quality_hori;`) forms, and neither is a struct, so they must be
        # consumed before the inline-struct handling below.
        #
        # The enumerator *names* are not carried into the message -- ROS
        # constants are per-message and these live several structs deep, where
        # names from different enums would collide. The numeric value is what
        # GPSd puts on the wire; gps.h remains the reference for what it means.
        enum_def = re.compile(r"\s*enum(?:\s+\w+)?\s*\{").match(body, index)
        if enum_def:
            _, cursor = brace_body(body, enum_def.end())
            tail = body.index(";", cursor)
            for decl in body[cursor:tail].split(","):
                decl = decl.strip()
                if decl:
                    name, array = parse_declarator(decl)
                    members.append(Member(name=name, ctype="enum", array=array))
            index = tail + 1
            continue

        # An anonymous union or struct with no declarator injects its members
        # into the enclosing scope (C11 6.7.2.1). gps_data_t uses exactly this
        # for the rtcm2/rtcm3/subframe/ais/raw/osc/version/error arms, so
        # splicing them in is both correct and what makes the scope filters and
        # the AIS exclusion match by plain member name.
        inline = re.compile(r"\s*(?:union|struct)(?:\s+(\w+))?\s*\{").match(
            body, index)
        if inline:
            inner_start = inline.end()
            depth, cursor = 1, inner_start
            while depth:
                if body[cursor] == "{":
                    depth += 1
                elif body[cursor] == "}":
                    depth -= 1
                cursor += 1
            tail = body.index(";", cursor)
            if not body[cursor:tail].strip():
                spliced = split_members(body[inner_start:cursor - 1])
                if inline.group(0).lstrip().startswith("union"):
                    # Only one of these is ever valid, exactly as for a named
                    # union, so they get the same 0-or-1 array treatment. Both
                    # gps_data_t's report union and rtcm2_t's arm union are
                    # declarator-less, so without this the representation would
                    # depend on whether GPSd happened to name the union.
                    for member in spliced:
                        member.union_arm = True
                members += spliced
                index = tail + 1
                continue

        anon = re.compile(r"\s*(struct|union)(?:\s+(\w+))?\s*\{").match(
            body, index)
        if anon:
            inner_start = anon.end()
            depth, cursor = 1, inner_start
            while depth:
                if body[cursor] == "{":
                    depth += 1
                elif body[cursor] == "}":
                    depth -= 1
                cursor += 1
            inner = body[inner_start:cursor - 1]
            tail = body.index(";", cursor)
            for decl in body[cursor:tail].split(","):
                decl = decl.strip()
                if not decl:
                    continue
                name, array = parse_declarator(decl)
                members.append(Member(name=name, ctype="struct", array=array,
                                      anon_body=inner, struct_tag=anon.group(2),
                                      is_union=(anon.group(1) == "union")))
            index = tail + 1
            continue

        semi = body.find(";", index)
        if semi == -1:
            break
        statement = " ".join(body[index:semi].split())
        index = semi + 1
        if not statement:
            continue
        # Only a real function pointer: `void (*update_fd)(int, bool)`.
        # Testing for a bare "(" also caught array extents that are
        # expressions, e.g. rtcm2_t's
        #   char message[(RTCM2_WORDS_MAX - 2) * sizeof(isgps30bits_t)]
        # which is an ordinary member and must fall through to be parsed.
        func_ptr = re.search(r"\(\s*\*\s*(\w+)\s*\)\s*\(", statement)
        if func_ptr:
            members.append(Member(name=func_ptr.group(1),
                                  ctype="function_pointer"))
            continue

        # The type is whatever precedes the first declarator, which is the
        # last identifier of the first comma-separated chunk. Splitting this
        # way copes with multi-word types ("unsigned char gnssid") and with
        # multi-declarator lines ("double x, y, z") without a keyword table.
        chunks = statement.split(",")
        head = re.match(r"^(.*?)([A-Za-z_]\w*)\s*(\[[^\]]*\])?$", chunks[0].strip())
        if not head:
            raise SystemExit(f"cannot parse struct member: {statement!r}")
        raw_type = head.group(1).strip()
        ctype = raw_type.rstrip("*").strip()
        if not ctype:
            raise SystemExit(f"cannot determine type of member: {statement!r}")
        members.append(Member(name=head.group(2), ctype=ctype,
                              array=(head.group(3)[1:-1] if head.group(3) else None),
                              is_pointer=raw_type.endswith("*")))
        for decl in chunks[1:]:
            decl = decl.strip()
            if not decl:
                continue
            name, array = parse_declarator(decl)
            members.append(Member(name=name, ctype=ctype, array=array,
                                  is_pointer=decl.lstrip().startswith("*")))
    return members


def parse_declarator(decl: str) -> Tuple[str, Optional[str]]:
    decl = decl.strip().lstrip("*").strip()
    match = re.match(r"^(\w+)\s*(\[([^\]]*)\])?$", decl)
    if not match:
        raise SystemExit(f"cannot parse declarator: {decl!r}")
    return match.group(1), (match.group(3) if match.group(2) else None)


# --------------------------------------------------------------------------
# Naming
# --------------------------------------------------------------------------

def snake_case(name: str) -> str:
    """GPSd member name -> a legal ROS field name.

    ROS field names must be lowercase alphanumeric with underscores. GPSd mixes
    conventions freely (PRN, altHAE, relPosN, errEllipseOrient, dgps_age), so
    the split has to handle acronym runs as well as ordinary camelCase:
    altHAE -> alt_hae, PRN -> prn, relPosN -> rel_pos_n.
    """
    out = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    out = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", out)
    out = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", out)
    out = re.sub(r"_+", "_", out).strip("_").lower()
    if not re.match(r"^[a-z][a-z0-9_]*$", out):
        raise SystemExit(f"member {name!r} does not map to a legal ROS name ({out!r})")
    return out


def camel(name: str) -> str:
    """GPSd member/tag name -> CamelCase fragment for a message name.

    Underscores must not survive: rosidl rejects them in message type names.
    `rtcm3_1001` becomes Rtcm31001, matching what message_base_name() produces
    for the same name used as a struct tag, so an arm gets the same spelling
    whether GPSd tagged its inline struct or not.

    Acronyms are *not* preserved: gps_fix_t::NED becomes Ned, not NED. rosidl
    normalises a run of capitals when it derives the C struct name (NED -> Ned)
    but emits the name as authored in the *referencing* message's header, so a
    name containing consecutive capitals produces two spellings that disagree
    and the generated C fails to compile with "unknown type name". Writing
    names already in rosidl's normalised form avoids the mismatch entirely.
    """
    return "".join(part.capitalize() for part in name.split("_") if part)


def message_base_name(cname: str) -> str:
    """C struct tag -> message name stem, e.g. gps_fix_t -> GPSDFix."""
    if cname == "gps_data_t":
        return MESSAGE_PREFIX + "Raw"
    stem = re.sub(r"_t$", "", cname)
    stem = re.sub(r"^gps_", "", stem)
    return MESSAGE_PREFIX + camel(stem)


def versioned(base: str, pair: Tuple[int, int]) -> str:
    """Append the API pair, keeping the boundary readable.

    The plain form is `<Stem><MAJOR>v<MINOR>` -- GPSDFix16v1 -- which is what
    the specified root name GPSDRaw<MAJOR>v<MINOR> uses.

    A stem that itself *ends in a digit* would run into the version and become
    ambiguous: the rtcm3 arm `rtcm3_1001` at API 9.0 would read
    GPSDRtcm3100 19v0 / GPSDRtcm31001 9v0 with no way to tell, and even the
    plain GPSDRtcm3 + 16v1 gives GPSDRtcm316v1. Those stems get a 'V'
    separator. rosidl rejects underscores in message names, so a letter is the
    only option.

    Applied only where the ambiguity exists, so the many stems that end in a
    letter keep the shorter, spec-matching form.
    """
    if base and base[-1].isdigit():
        return f"{base}V{pair[0]}v{pair[1]}"
    return f"{base}{pair[0]}v{pair[1]}"


# --------------------------------------------------------------------------
# Type mapping (see the module docstring for the authoritative table)
# --------------------------------------------------------------------------

SCALAR_TYPES = {
    "double": "float64",
    "float": "float32",
    "bool": "bool",
    "char": "int8",
    "signed char": "int8",
    "signed int": "int32",
    "signed short": "int16",
    "signed short int": "int16",
    "signed long": "int64",
    "signed long int": "int64",
    "signed long long": "int64",
    "unsigned long int": "uint64",
    "long double": "float64",   # widened; ROS has no 80/128-bit float
    "unsigned char": "uint8",
    "short": "int16",
    "short int": "int16",
    "unsigned short": "uint16",
    "unsigned short int": "uint16",
    "int": "int32",
    "signed": "int32",
    "unsigned": "uint32",
    "unsigned int": "uint32",
    "long": "int64",
    "long int": "int64",
    "unsigned long": "uint64",
    "long long": "int64",
    "unsigned long long": "uint64",
    "int8_t": "int8",
    "uint8_t": "uint8",
    "int16_t": "int16",
    "uint16_t": "uint16",
    "int32_t": "int32",
    "uint32_t": "uint32",
    "int64_t": "int64",
    "uint64_t": "uint64",
    "size_t": "uint64",
    "time_t": "int64",          # seconds, but may be a duration or TOW
    "gps_mask_t": "uint64",     # verbatim, undecoded
    "gnssid_t": "uint8",
    "gps_fd_t": "int32",
    "watch_t": "uint32",        # typedef uint32_t; a WATCH_* bitmask
    "isgps30bits_t": "uint32",  # typedef uint32_t; a raw RTCM2 30-bit word
    "enum": "int32",            # anonymous enum member; see split_members
    "socket_t": "int32",
    "timestamp_t": "float64",   # pre-API-9 leftover
}

TIMESPEC_TYPES = {"timespec_t", "struct timespec"}


@dataclass
class Field:
    ros_type: str
    name: str
    comment: str = ""
    # How the parser should fill it; see emit_parser().
    kind: str = "scalar"          # scalar | string | bytes | time | struct
    c_expr: str = ""              # C member path relative to its parent
    array: bool = False
    union_arm: bool = False       # a 0-or-1 array standing in for a union arm
    # Set by flatten_model(), for the flat root only.
    path: Tuple[str, ...] = ()    # C member names, outermost first
    gate: str = ""                # set-mask bit guarding a union arm's data
    group: str = ""               # parallel-array group this leaf belongs to


class Model:
    """The field model for one API pair, plus the structs it reached."""

    def __init__(self, pair: Tuple[int, int], src: str):
        self.pair = pair
        self.src = src
        self.messages: Dict[str, List[Field]] = {}
        self.order: List[str] = []
        self.mapped: Dict[str, List[str]] = {}   # struct -> mapped member names
        self.skipped: Dict[str, List[str]] = {}  # struct -> excluded members

    def message(self, name: str) -> List[Field]:
        if name not in self.messages:
            self.messages[name] = []
            self.order.append(name)
        return self.messages[name]


def build_model(pair: Tuple[int, int], src: str, scope_members: Sequence[str]) -> Model:
    model = Model(pair, src)
    body = find_struct_body(src, "gps_data_t")
    if body is None:
        raise SystemExit(f"gps_data_t not found at {REFERENCE_REVS[pair]}")

    root = versioned(MESSAGE_PREFIX + "Raw", pair)
    fields = model.message(root)
    fields.append(Field(ros_type="std_msgs/Header", name="header",
                        comment="", kind="header"))

    members = split_members(resolve_conditionals(body))
    mapped, skipped = [], []
    for member in members:
        if member.name in EXCLUDED_MEMBERS:
            skipped.append(member.name)
            continue
        if member.name not in scope_members:
            continue
        if member.name in STANDALONE_MEMBER_NAMES:
            # Generated as its own root below, not as a field here.
            continue
        before = len(fields)
        add_field(model, fields, member, parent="gps_data_t")
        if member.union_arm:
            for f in fields[before:]:
                if not f.ros_type.endswith("[]"):
                    f.ros_type += "[]"
                f.array = True
                f.union_arm = True
        mapped.append(member.name)
    model.mapped["gps_data_t"] = mapped
    model.skipped["gps_data_t"] = skipped

    # The standalone roots. Generated even though they are no longer fields of
    # GPSDRaw, and given their own Header since they are published in their
    # own right and a subscriber needs a stamp and frame.
    for tag in STANDALONE_ROOTS:
        body = find_struct_body(src, tag)
        if body is None:
            continue
        name = versioned(message_base_name(tag), pair)
        if name not in model.messages:
            emit_struct(model, name,
                        split_members(resolve_conditionals(body)), tag)
        root_fields = model.messages[name]
        if not any(f.name == "header" for f in root_fields):
            root_fields.insert(0, Field(ros_type="std_msgs/Header",
                                        name="header", kind="header"))
    return model


def add_field(model: Model, fields: List[Field], member: Member, parent: str) -> None:
    if member.is_pointer:
        """Pointers are never published, whatever they point at.

        The motivating case is fixsource_t, whose server/server_ip/port/device
        are `const char *` aimed at the caller's own memory: gps_open() stores
        the host and port arguments verbatim (libgps/libgps_core.c), and
        gpsd_client passes `host.c_str()` from a std::string local to start(),
        so those pointers dangle as soon as start() returns. Dereferencing them
        to build a message would be undefined behaviour in this very package.

        Nothing is lost here: fixsource_t::spec carries the same information as
        a real char array, and it is published.
        """
        model.skipped.setdefault(parent, []).append(member.name)
        return

    name = snake_case(member.name)
    if any(existing.name == name for existing in fields):
        raise SystemExit(
            f"{parent}.{member.name} collides with an existing ROS field {name!r}")

    ctype = member.ctype

    # Inline anonymous struct: name the message after parent + member.
    if member.anon_body is not None:
        if member.struct_tag:
            # `struct gps_rangesat_t { ... } sat[15];` -- defined inline but
            # tagged, so name it as if it were declared at file scope.
            sub = versioned(message_base_name(member.struct_tag), model.pair)
        else:
            # `parent` may be a dotted path (rtcm3_t.rtcmtypes) rather than a
            # struct tag, so take only its last component and strip the _t.
            stem = parent.split(".")[-1]
            base = message_base_name(stem).replace(MESSAGE_PREFIX, "", 1)
            sub = versioned(f"{MESSAGE_PREFIX}{base}{camel(member.name)}",
                            model.pair)
        if sub in model.messages:
            # Already emitted from another use of the same tag.
            fields.append(Field(ros_type=sub + ("[]" if member.array else ""),
                                name=name, kind="struct", c_expr=member.name,
                                array=bool(member.array)))
            return
        emit_struct(model, sub,
                    split_members(resolve_conditionals(member.anon_body)),
                    f"{parent}.{member.name}", as_union=member.is_union)
        fields.append(Field(ros_type=sub + ("[]" if member.array else ""),
                            name=name, kind="struct", c_expr=member.name,
                            array=bool(member.array)))
        return

    if ctype.startswith("struct "):
        tag = ctype.split()[1]
        sub_body = find_struct_body(model.src, tag)
        if sub_body is None:
            raise SystemExit(f"{parent}.{member.name}: struct {tag} not found")
        sub = versioned(message_base_name(tag), model.pair)
        if sub not in model.messages:
            emit_struct(model, sub,
                        split_members(resolve_conditionals(sub_body)), tag)
        fields.append(Field(ros_type=sub + ("[]" if member.array else ""),
                            name=name, kind="struct", c_expr=member.name,
                            array=bool(member.array)))
        return

    if ctype in TIMESPEC_TYPES:
        fields.append(Field(ros_type="builtin_interfaces/Time", name=name,
                            kind="time", c_expr=member.name))
        return

    if ctype == "function_pointer":
        return

    if ctype.startswith("enum "):
        # `enum RTCM3_QUALITY_INDICATOR_TRANSFORMATION quality_hori;` -- a
        # reference to a tagged enum, which is still just an integer.
        ctype = "enum"

    if SCALAR_TYPES.get(ctype) is None:
        # A typedef naming a struct, e.g. `typedef struct orbit orbit_t;`
        # (subframe_t::orbit and ::orbit1 are declared as bare orbit_t).
        # Checked after SCALAR_TYPES so scalar typedefs such as watch_t and
        # isgps30bits_t keep their integer mapping.
        tag = struct_typedefs(model.src).get(ctype)
        if tag is not None:
            sub_body = find_struct_body(model.src, tag)
            if sub_body is None:
                raise SystemExit(
                    f"{parent}.{member.name}: {ctype} names struct {tag}, "
                    f"which was not found")
            sub = versioned(message_base_name(ctype), model.pair)
            if sub not in model.messages:
                emit_struct(model, sub,
                            split_members(resolve_conditionals(sub_body)), tag)
            fields.append(Field(ros_type=sub + ("[]" if member.array else ""),
                                name=name, kind="struct", c_expr=member.name,
                                array=bool(member.array)))
            return

    ros = SCALAR_TYPES.get(ctype)
    if ros is None:
        raise SystemExit(
            f"{parent}.{member.name}: no ROS mapping for C type {ctype!r}. "
            "Add it to SCALAR_TYPES with a documented rationale.")

    if member.array:
        # GPSd draws the text/bytes line itself: `char[N]` is always a
        # NUL-terminated string (paths, driver names, RTCM2 type-16 ASCII
        # messages), while a raw payload is `unsigned char[N]` -- rtcm3_t's
        # 1024-byte `data` is the clearest case. Mapping char[N] to string and
        # letting unsigned char[N] fall through to uint8[] follows the header
        # rather than guessing from field names.
        #
        # The fill side uses strnlen with sizeof, so an array that happens to
        # be full with no NUL is truncated rather than overrun.
        if ctype == "char":
            fields.append(Field(ros_type="string", name=name, kind="string",
                                c_expr=member.name))
        else:
            fields.append(Field(ros_type=ros + "[]", name=name, kind="scalar",
                                c_expr=member.name, array=True))
        return

    fields.append(Field(ros_type=ros, name=name, kind="scalar",
                        c_expr=member.name))


def flatten_model(model: Model) -> None:
    """Collapse the nested messages into one flat root, in place.

    Every project-defined sub-message is inlined: its fields join the root
    under a path-prefixed name (`fix` + `time` -> `fix_time`). Only standard
    ROS types stay nested. Path prefixes are not cosmetic -- bare leaf names
    collide 63 times in one API version, since `time`, `status` and `temp`
    each appear in several structs.

    Two rules on array-ness:

    * A real array (`skyview`, `imu`, `devices.list`, `raw.meas`) turns each
      leaf below it into a parallel array. Members of one group always have the
      same length, which the generated fill guarantees by resizing and writing
      them from a single loop.
    * A union arm contributes nothing. Its leaves stay scalars and the `set`
      mask says whether they mean anything -- the same contract the mask
      already carries for AIS. Making them 0-or-1 arrays instead would force
      `msg.osc_delta[0]` on every reader for no added information.
    """
    root = versioned(MESSAGE_PREFIX + "Raw", model.pair)
    flat: List[Field] = []

    def base(ros_type: str) -> str:
        return ros_type[:-2] if ros_type.endswith("[]") else ros_type

    def walk(message_name: str, prefix: str, path: Tuple[str, ...],
             group: str, gate: str) -> None:
        for f in model.messages[message_name]:
            if f.kind == "header":
                if not prefix:
                    flat.append(f)
                continue
            name = f"{prefix}{f.name}"
            hop = path + (f.c_expr,) if f.c_expr else path
            child = base(f.ros_type)
            # A real array (not a union arm) opens a new parallel-array group.
            inner = name if (f.array and not f.union_arm) else group
            # A union arm's data is only readable when the mask names it;
            # reading any other arm is a read of an inactive union member.
            gated = gate or (REPORT_UNION_BITS.get(f.c_expr, "")
                             if f.union_arm else "")
            if child in model.messages:
                walk(child, name + "_", hop, inner, gated)
            else:
                flat.append(Field(
                    ros_type=child + ("[]" if inner else ""),
                    name=name, comment=f.comment, kind=f.kind,
                    c_expr=".".join(hop), array=bool(inner),
                    union_arm=False, path=hop, gate=gated, group=inner))

    walk(root, "", (), "", "")
    model.messages = {root: flat}
    model.order = [root]


def emit_struct(model: Model, message_name: str, members: List[Member],
                parent: str, as_union: bool = False) -> None:
    """Emit one message for a struct, or for a union's arms.

    A union's arms become **0-or-1 element arrays** rather than plain fields.
    ROS has no variant type, and a flat message would serialise all 26 rtcm3
    arms on every report -- roughly 180 dead fields of wire and CPU per RTCM3
    message -- while also requiring the reader to know the discriminator to
    tell which one means anything. As arrays, an inactive arm costs the four
    bytes of its length, and `!msg.rtcm3_1005.empty()` says outright that the
    report is a type 1005.

    It also keeps the parser from reading an inactive union member, which is
    undefined behaviour, not merely wasteful.
    """
    fields = model.message(message_name)
    mapped, skipped = [], []
    for member in members:
        if member.name in EXCLUDED_MEMBERS:
            skipped.append(member.name)
            continue
        before = len(fields)
        add_field(model, fields, member, parent=parent)
        if as_union or member.union_arm:
            for f in fields[before:]:
                # An arm that is already an array (rtcm3's raw `data`) needs no
                # wrapping: ROS forbids nested arrays, and its emptiness
                # already signals an inactive arm.
                if not f.ros_type.endswith("[]"):
                    f.ros_type += "[]"
                f.array = True
                f.union_arm = True
    model.mapped[parent] = mapped
    if skipped:
        model.skipped[parent] = skipped


# --------------------------------------------------------------------------
# The one message with no version suffix.
#
# Every other message mirrors a struct whose shape changes with the GPSd API,
# so it carries the pair in its name. This one carries an opaque string, which
# does not change shape, so a single type serves every API version.
#
# Emitted here rather than checked in by hand because orphans() owns the whole
# msg/ directory: a hand-written file there would be deleted by the next run.
JSON_MESSAGE = """\
# Generated by tools/generate_raw_msgs.py.
# Do not edit; edit the generator and regenerate.
#
# One GPSd JSON report exactly as libgps handed it back, including report
# classes libgps cannot decode into gps_data_t.
#
# Unversioned on purpose: the payload is a string, so its shape does not vary
# with the GPSd C API the way the GPSDRaw types do.

# Stamped when gps_read() returned the line, not when the cycle published.
std_msgs/Header header

string json
"""

# The `set` mask constants
# --------------------------------------------------------------------------

_STRUCT_TYPEDEFS: Dict[int, Dict[str, str]] = {}


def struct_typedefs(src: str) -> Dict[str, str]:
    """{typedef name: struct tag} for `typedef struct <tag> <name>;` forms.

    Cached per source text; the lookup is only consulted for types that are
    not already scalars, so it never shadows watch_t or isgps30bits_t.
    """
    key = id(src)
    if key not in _STRUCT_TYPEDEFS:
        _STRUCT_TYPEDEFS[key] = {
            name: tag for tag, name in
            re.findall(r"typedef\s+struct\s+(\w+)\s+(\w+)\s*;", src)
        }
    return _STRUCT_TYPEDEFS[key]


def mask_constants(src: str) -> List[Tuple[str, str]]:
    """`<NAME>_SET (1llu<<N)` -> ('SET_<NAME>', value), plus SET_UNION.

    Renamed because rosidl emits constants as static constexpr members while
    gps.h defines the GPSd spellings as global macros; a member named
    STATUS_SET in a header parsed after gps.h is destroyed by the preprocessor.
    gps.h defines no SET_* macros, so the flipped form is collision-free.
    """
    out = []
    for match in re.finditer(
            r"^#define\s+([A-Z0-9_]+)_SET\s+\(1u?ll?u?\s*<<\s*(\d+)\)", src, re.M):
        out.append((f"SET_{match.group(1)}", str(1 << int(match.group(2)))))
    # UNION_SET is a composite of the arm bits rather than a shift, so it has
    # to be resolved by OR-ing the constants it names. It deliberately keeps
    # AIS_SET even though AIS is not published: the constant must mean
    # what gps.h says it means.
    bits = {name: int(value) for name, value in out}
    union = re.search(r"^#define\s+UNION_SET\s+\((.*?)\)", src, re.M | re.S)
    if union:
        value = 0
        for token in re.findall(r"([A-Z0-9_]+)_SET", union.group(1)):
            key = f"SET_{token}"
            if key not in bits:
                raise SystemExit(f"UNION_SET names unknown bit {token}_SET")
            value |= bits[key]
        out.append(("SET_UNION", str(value)))

    # Emitted as SET_HIGHEST_BIT, not gps.h's own SET_HIGH_BIT spelling.
    # gps.h defines SET_HIGH_BIT as a plain macro, so a message constant of
    # that name is destroyed by the preprocessor in any translation unit that
    # sees gps.h -- the exact hazard the rename exists to avoid, which the
    # <NAME>_SET -> SET_<NAME> rule happens not to cover because this one is
    # already spelled SET_*. assert_no_macro_collisions() below is the general
    # guard; this is the one name it forced us to change.
    high = re.search(r"^#define\s+SET_HIGH_BIT\s+(\d+)", src, re.M)
    if high:
        out.append(("SET_HIGHEST_BIT", high.group(1)))
    return out


def macro_names(src: str) -> set:
    """Every object-like macro gps.h defines."""
    return set(re.findall(r"^#define\s+([A-Za-z_]\w*)", src, re.M))


def assert_no_macro_collisions(constants, src: str, rev: str) -> None:
    """Fail if any emitted constant shares a name with a gps.h macro.

    rosidl emits message constants as `static constexpr` members. If gps.h has
    already defined that name as a macro, the member declaration is mangled by
    the preprocessor and the build fails somewhere confusing -- and downstream
    users, who control neither include order nor our naming, cannot work around
    it. Checking the whole macro namespace beats maintaining a list of the
    spellings we happen to know about.
    """
    macros = macro_names(src)
    clashes = sorted(name for name, _ in constants if name in macros)
    if clashes:
        raise SystemExit(
            f"{rev}: generated message constants collide with gps.h macros: "
            f"{', '.join(clashes)}. Rename them; a constant whose name is a "
            f"gps.h macro cannot be used by any translation unit that "
            f"includes gps.h.")


# --------------------------------------------------------------------------
# Emission
# --------------------------------------------------------------------------

BANNER = ("# Generated by tools/generate_raw_msgs.py from GPSd {rev} "
          "(libgps API {major}.{minor}).\n# Do not edit; edit the generator "
          "and regenerate.\n")


def emit_msg(model: Model, name: str, rev: str,
             constants: Sequence[Tuple[str, str]]) -> str:
    major, minor = model.pair
    lines = [BANNER.format(rev=rev, major=major, minor=minor)]
    if name.startswith(MESSAGE_PREFIX + "Raw"):
        lines.append(
            f"# Raw GPSd report (gps_data_t) as delivered by libgps API "
            f"{major}.{minor}.\n")
    for const_name, value in constants:
        lines.append(f"uint64 {const_name} = {value}")
    if constants:
        lines.append("")
    for f in model.messages[name]:
        lines.append(f"{f.ros_type} {f.name}")
    return "\n".join(lines).rstrip() + "\n"


def emit_mask_asserts(model: Model, constants) -> List[str]:
    """static_assert every SET_<NAME> against the gps.h macro it came from.

    The generator flips GPSd's `<NAME>_SET` to `SET_<NAME>` so the constants survive the
    preprocessor. The flip is mechanical, which makes it exactly the kind of
    thing that can go quietly wrong: a message whose SET_LATLON does not equal
    gps.h's LATLON_SET is worse than useless, because every mask test written
    against it silently checks the wrong bit.

    These land in the fill header rather than in a test so they are checked in
    every build that publishes raw messages, not only when BUILD_TESTING is on
    -- and at compile time, where a mismatch cannot be observed as a flake.

    Two things are deliberately not asserted:

    * Each is wrapped in `#ifdef`. The messages are generated from the last rev
      at a pair, but a build can use an *earlier* rev reporting the same pair,
      where a late-added bit does not exist yet. Naming an undefined macro
      would be a compile error rather than graceful degradation
      the struct members.
    * SET_HIGHEST_BIT is skipped. It is a count of bits rather than a bit, and
      it is the one value that genuinely moves within an API pair -- GPSd 3.24
      and 3.26.1 are both API 14.0 with different counts -- so there is no
      fixed value to assert. test_gpsd_raw_include_order.cpp bounds it instead.
    """
    root = versioned(MESSAGE_PREFIX + "Raw", model.pair)
    out = [
        "// The message's mask constants are GPSd's own bit values under a",
        "// name the preprocessor leaves alone. Checked here, where both",
        "// spellings are legitimately in scope, so a rename that changes a",
        "// value cannot reach a subscriber.",
    ]
    for name, value in constants:
        if name == "SET_HIGHEST_BIT":
            continue    # a count, not a bit; moves within a pair. See above.
        gpsd_name = "UNION_SET" if name == "SET_UNION" else name[len("SET_"):] + "_SET"
        out += [
            f"#ifdef {gpsd_name}",
            f"static_assert({PACKAGE}::msg::{root}::{name} == "
            f"static_cast<uint64_t>({gpsd_name}),",
            f"              \"{name} disagrees with gps.h's {gpsd_name}\");",
            "#endif",
        ]
    out.append("")
    return out


def emit_parser(model: Model, rev: str, constants=()) -> str:
    """Per-pair fill functions, guarded on the exact API pair.

    Every assignment is wrapped in the member-detection idiom: an API pair
    spans a range of header states, so a message generated from the pair's last
    rev can name members an older libgps reporting the same pair lacks.
    """
    major, minor = model.pair
    root = versioned(MESSAGE_PREFIX + "Raw", model.pair)
    out = [
        f"// Generated by tools/generate_raw_msgs.py from GPSd {rev} "
        f"(libgps API {major}.{minor}).",
        "// Do not edit; edit the generator and regenerate.",
        f"#ifndef GPSD_CLIENT__PARSERS__GENERATED__FILL_{major}V{minor}_HPP_",
        f"#define GPSD_CLIENT__PARSERS__GENERATED__FILL_{major}V{minor}_HPP_",
        "",
        "#include <gpsd_client/parsers/generated/gpsd_has_member.hpp>",
        "",
        f"#include <{PACKAGE}/msg/{ros_header_name(root)}.hpp>",
    ]
    for name in model.order:
        if name != root:
            out.append(f"#include <{PACKAGE}/msg/{ros_header_name(name)}.hpp>")
    out += [
        "",
        "#include <gps.h>",
        "",
        "#include <cstddef>",
        "#include <cstring>",
        "#include <iterator>",
        "#include <type_traits>",
        "#include <vector>",
        "",
        "// Which pair this build actually uses. gpsd_raw_message.hpp sets these",
        "// before including one fill header, so a libgps newer than anything",
        "// tested can still be pointed at the newest parser. Defaulted here so",
        "// the header stays usable on its own.",
        "#ifndef GPSD_RAW_FILL_MAJOR",
        "#define GPSD_RAW_FILL_MAJOR GPSD_API_MAJOR_VERSION",
        "#endif",
        "#ifndef GPSD_RAW_FILL_MINOR",
        "#define GPSD_RAW_FILL_MINOR GPSD_API_MINOR_VERSION",
        "#endif",
        "",
        f"#if GPSD_RAW_FILL_MAJOR == {major} && GPSD_RAW_FILL_MINOR == {minor}",
        "",
        "namespace gpsd_client",
        "{",
        "namespace generated",
        "{",
        "",
    ]

    if constants:
        out += emit_mask_asserts(model, constants)

    # Member-detection traits, one per distinct C member *segment*. Paths are
    # flattened now, so `fix.ecef.x` needs traits for fix, ecef and x -- one
    # per hop, since each hop is guarded separately.
    names = sorted({seg for fields in model.messages.values()
                    for f in fields for seg in f.path})
    for member_name in names:
        out.append(f"GPSD_DEFINE_HAS_MEMBER({member_name})")
    out.append("")

    # Forward-declare every overload first. The root's fill() calls fill() on
    # its sub-structs, and unqualified lookup in a template happens at
    # definition time -- ADL cannot find these, since the arguments live in ::
    # and the message package while the overloads live in gpsd_client::generated.
    out.append("// Forward declarations; see the note in the generator.")
    for name in model.order:
        out.append(f"template <typename T>")
        out.append(f"inline void fill(const T& in, {PACKAGE}::msg::{name}& out);")
    out.append("")

    for name in model.order:
        out += emit_fill_function(model, name)
    out += [
        "}  // namespace generated",
        "}  // namespace gpsd_client",
        "",
        f"#endif  // GPSD_RAW_FILL_MAJOR == {major} ...",
        "",
        f"#endif  // GPSD_CLIENT__PARSERS__GENERATED__FILL_{major}V{minor}_HPP_",
        "",
    ]
    return "\n".join(out)


def emit_fill_function(model: Model, message_name: str) -> List[str]:
    """The flat root's fill, plus one filler per parallel-array group.

    Scalars are emitted as a tree over their C paths, so `fix.ecef.x` and
    `fix.ecef.y` share one `if constexpr (has_fix<T>)` and one
    `if constexpr (has_ecef<...>)` rather than repeating the chain per leaf.
    Every hop is guarded, because an API pair spans a range of header states.

    Union arms are gated on the set mask as well. Their leaves are plain
    scalars now, but the underlying C members still share storage, so reading
    an arm the mask does not name is a read of an inactive union member.
    """
    out = [
        "template <typename T>",
        f"inline void fill(const T& in, {PACKAGE}::msg::{message_name}& out)",
        "{",
        "  (void)in;",
        "  (void)out;",
    ]
    scalars = [f for f in model.messages[message_name]
               if f.kind != "header" and not f.group]
    out += emit_path_tree(scalars, 0, "in", "T", "  ")
    out += ["}", ""]

    for group in group_names(model, message_name):
        out += emit_group_fill(model, message_name, group)
    return out


def group_names(model: Model, message_name: str) -> List[str]:
    seen: List[str] = []
    for f in model.messages[message_name]:
        if f.group and f.group not in seen:
            seen.append(f.group)
    return seen


def leaf_assign(f: Field, expr: str, target: str) -> List[str]:
    """The assignment itself, once every guard is open."""
    if f.kind == "time":
        return [f"{target}.sec = static_cast<int32_t>({expr}.tv_sec);",
                f"{target}.nanosec = static_cast<uint32_t>({expr}.tv_nsec);"]
    if f.kind == "string":
        return [f"{target}.assign({expr}, strnlen({expr}, sizeof({expr})));"]
    if f.kind == "bytes" or (f.array and f.kind == "scalar" and not f.group):
        return [f"{target}.assign(std::begin({expr}), std::end({expr}));"]
    return [f"{target} = {expr};"]


def emit_path_tree(fields: List[Field], depth: int, in_expr: str,
                   tname: str, indent: str) -> List[str]:
    """Nested `if constexpr` blocks mirroring the C member paths."""
    out: List[str] = []
    buckets: Dict[str, List[Field]] = {}
    for f in fields:
        buckets.setdefault(f.path[depth], []).append(f)

    for seg, group in buckets.items():
        leaves = [f for f in group if len(f.path) == depth + 1]
        deeper = [f for f in group if len(f.path) > depth + 1]
        expr = f"{in_expr}.{seg}"
        out.append(f"{indent}if constexpr (has_{seg}<{tname}>::value) {{")
        body_indent = indent + "  "

        # A union arm's storage is shared; only the arm the mask names may be
        # read. Emitted once for the whole arm rather than per leaf.
        gate = group[0].gate if depth == 0 and group[0].gate else ""
        if gate and all(f.gate == gate for f in group):
            out.append(f"{body_indent}if (0 != (in.set & {gate})) {{")
            body_indent += "  "

        for f in leaves:
            for line in leaf_assign(f, expr, f"out.{f.name}"):
                out.append(f"{body_indent}{line}")
        if deeper:
            alias = "T_" + "_".join(deeper[0].path[:depth + 1])
            out.append(f"{body_indent}using {alias} = "
                       f"std::decay_t<decltype({expr})>;")
            out += emit_path_tree(deeper, depth + 1, expr, alias, body_indent)

        if gate and all(f.gate == gate for f in group):
            out.append(f"{indent}  }}")
        out.append(f"{indent}}}")
    return out


def emit_group_fill(model: Model, message_name: str, group: str) -> List[str]:
    """One filler per parallel-array group, resizing and writing in one loop.

    Every array in a group therefore has the same length by construction --
    unequal lengths are not representable rather than merely untested.

    The caller passes *source indices* rather than a count, because only it
    knows which elements are valid and the rule differs per group. skyview,
    devices.list and imu take a prefix; rawdata_t::meas does not -- GPSd fills
    it at arbitrary indices and marks unused entries with an svid of 0 or 255,
    so its valid set is a filtered subset. Indices express both.
    """
    fields = [f for f in model.messages[message_name] if f.group == group]
    # The C path down to the array, and the member path below each element.
    array_depth = len(group.split("_"))
    lead = fields[0].path[:array_depth]
    out = [
        f"/// Fill the {group}_* arrays from in.{'.'.join(lead)}, taking the",
        "/// elements named by idx. One loop, so every array in the group ends",
        "/// the same length.",
        "template <typename T>",
        f"inline void fill_{group}(const T& in, "
        f"{PACKAGE}::msg::{message_name}& out,",
        "                          const std::vector<std::size_t>& idx)",
        "{",
        "  (void)in;",
        "  (void)out;",
        "  (void)idx;",
        "  const std::size_t count = idx.size();",
    ]
    guards = []
    expr = "in"
    tname = "T"
    for i, seg in enumerate(lead):
        guards.append(f"  if constexpr (has_{seg}<{tname}>::value) {{")
        expr = f"{expr}.{seg}"
        tname = "T_" + "_".join(lead[:i + 1])
        guards.append(f"    using {tname} = std::decay_t<decltype({expr})>;")
    out += guards
    gate = fields[0].gate
    body = "    "
    if gate:
        out.append(f"    if (0 != (in.set & {gate})) {{")
        body = "      "
    for f in fields:
        out.append(f"{body}out.{f.name}.resize(count);")
    out.append(f"{body}for (std::size_t i = 0; i < count; ++i) {{")
    out.append(f"{body}  const std::size_t src = idx[i];")
    for f in fields:
        elem = f"{expr}[src]" + "".join(f".{p}" for p in f.path[array_depth:])
        inner = "T_elem_" + f.name
        out.append(f"{body}  using {inner} = "
                   f"std::decay_t<decltype({expr}[src])>;")
        out.append(f"{body}  if constexpr (has_{f.path[array_depth]}"
                   f"<{inner}>::value) {{")
        for line in leaf_assign(f, elem, f"out.{f.name}[i]"):
            out.append(f"{body}    {line}")
        out.append(f"{body}  }}")
    out.append(f"{body}}}")
    if gate:
        out.append("    }")
    for _ in lead:
        out.append("  }")
    out += ["}", ""]
    return out


def ros_header_name(message_name: str) -> str:
    """GPSDRaw16v1 -> gpsd_raw16v1, matching rosidl's generated header names.

    rosidl uses the same camel-to-snake rule as ROS field names, including the
    acronym-run split that turns GPSDBaseline into gpsd_baseline rather than
    gpsdbaseline. Verified against the headers rosidl actually emitted for all
    seven core fix messages.
    """
    return snake_case(message_name)


HAS_MEMBER_HEADER = """\
// Generated by tools/generate_raw_msgs.py. Do not edit.
#ifndef GPSD_CLIENT__PARSERS__GENERATED__GPSD_HAS_MEMBER_HPP_
#define GPSD_CLIENT__PARSERS__GENERATED__GPSD_HAS_MEMBER_HPP_

#include <type_traits>
#include <utility>

/// Compile-time detection of a struct member.
///
/// A GPSd API pair names a *range* of header states, not one: gps_fix_t gains
/// ant_stat, clockbias, clockdrift, jam, temp and wtemp between GPSd 3.24 and
/// 3.26.1, which both report API 14.0. Messages are generated from the last
/// rev of a pair, so generated code can name members an older libgps reporting
/// the same pair does not have.
///
/// CMake cannot help here -- check_cxx_symbol_exists() does not see struct
/// members -- so detection happens in C++ and each assignment is guarded by
/// `if constexpr`, leaving the message field at its default when absent.
#define GPSD_DEFINE_HAS_MEMBER(name)                                       \\
  template <typename T, typename = void>                                   \\
  struct has_##name : std::false_type {};                                  \\
  template <typename T>                                                    \\
  struct has_##name<T, std::void_t<decltype(std::declval<T&>().name)>>     \\
      : std::true_type {};

#endif  // GPSD_CLIENT__PARSERS__GENERATED__GPSD_HAS_MEMBER_HPP_
"""


def emit_selection_ladder() -> str:
    """gpsd_raw_message.hpp -- the single compile-time selection point for raw.

    One `using GpsdRawMsg = ...` and one fill header, chosen from the libgps
    the workspace is built against. Everything downstream (the parser, the
    factory, client.cpp) names GpsdRawMsg and needs no version logic of its own.

    The policy matches gpsd_parser_factory.cpp: hard error below API 9, and for
    a pair newer than anything generated, warn and fall back to the newest --
    which is why the fill headers take GPSD_RAW_FILL_MAJOR/MINOR as an override
    rather than testing GPSD_API_*_VERSION directly.
    """
    pairs = sorted(REFERENCE_REVS)
    newest = pairs[-1]
    out = [
        "// Generated by tools/generate_raw_msgs.py. Do not edit.",
        "#ifndef GPSD_CLIENT__GPSD_RAW_MESSAGE_HPP_",
        "#define GPSD_CLIENT__GPSD_RAW_MESSAGE_HPP_",
        "",
        "// GPSD_RAW_MESSAGE_NAME names the selected message for log output, so",
        "// an operator can see which version was compiled in without guessing.",
        "//",
        "// gps.h defines STATUS_* macros that collide with the ROS message",
        "// constants, so a message header must never be parsed after it. Every",
        "// branch below includes its messages before gps.h is reached.",
        "#include <gps.h>",
        "",
        "#if GPSD_API_MAJOR_VERSION < 9",
        '#error "gpsd_client requires GPSd API version >= 9 (GPSd >= 3.20)"',
        "#endif",
        "",
    ]
    for index, pair in enumerate(pairs):
        major, minor = pair
        test = (f"GPSD_API_MAJOR_VERSION == {major} && "
                f"GPSD_API_MINOR_VERSION == {minor}")
        out.append(f"#{'if' if index == 0 else 'elif'} {test}")
        out.append(f"#define GPSD_RAW_FILL_MAJOR {major}")
        out.append(f"#define GPSD_RAW_FILL_MINOR {minor}")
    out += [
        "#else",
        "// Newer than anything this generator knows about. Use the newest",
        "// available message; its fields are a subset of what the build's",
        "// gps.h declares, and every generated assignment is member-guarded,",
        "// so this compiles -- it just cannot carry members added later.",
        f'#warning "Untested GPSd API version; falling back to the API '
        f'{newest[0]}.{newest[1]} raw message"',
        f"#define GPSD_RAW_FILL_MAJOR {newest[0]}",
        f"#define GPSD_RAW_FILL_MINOR {newest[1]}",
        "#endif",
        "",
    ]
    for index, pair in enumerate(pairs):
        major, minor = pair
        name = versioned(MESSAGE_PREFIX + "Raw", pair)
        guard = (f"GPSD_RAW_FILL_MAJOR == {major} && "
                 f"GPSD_RAW_FILL_MINOR == {minor}")
        out.append(f"#{'if' if index == 0 else 'elif'} {guard}")
        out.append(f"#include <gpsd_client/parsers/generated/"
                   f"gpsd_raw_fill_{major}v{minor}.hpp>")
        for tag in STANDALONE_ROOTS:
            out.append(f"#include <{PACKAGE}/msg/"
                       f"{ros_header_name(versioned(message_base_name(tag), pair))}"
                       f".hpp>")
        out.append(f'#define GPSD_RAW_MESSAGE_NAME "{name}"')
        out.append("namespace gpsd_client")
        out.append("{")
        out.append(f"using GpsdRawMsg = {PACKAGE}::msg::{name};")
        # One alias per standalone root, driven by STANDALONE_ROOTS rather
        # than a second hardcoded list -- so emptying that tuple removes the
        # aliases too, instead of leaving them pointing at deleted messages.
        for tag in STANDALONE_ROOTS:
            base = message_base_name(tag)
            out.append(f"using Gpsd{base[len('GPSD'):]}Msg = {PACKAGE}::msg::"
                       f"{versioned(base, pair)};")
        out.append("}  // namespace gpsd_client")
    out += [
        "#endif",
        "",
        "#endif  // GPSD_CLIENT__GPSD_RAW_MESSAGE_HPP_",
        "",
    ]
    return "\n".join(out)


def emit_version_workflow(pair: Tuple[int, int], rev: str) -> str:
    """One GitHub Actions workflow per API pair.

    Each pair gets its own workflow file rather than being a row in a matrix,
    so it appears in the Actions list under its own name with its own badge,
    run history and re-run button -- a failure names the API version without
    anyone opening a matrix job. All the logic lives once, in the reusable
    .github/workflows/gpsd_api_shared.yml that these call.

    Generated from REFERENCE_REVS so that adding an API pair creates its
    workflow too; `--check` fails if the checked-in set has drifted.
    """
    major, minor = pair
    message = versioned(MESSAGE_PREFIX + "Raw", pair)
    unreleased = not rev.startswith("release-")
    note = ("#\n"
            "# This API pair shipped in no GPSd release, so the revision below\n"
            "# is a bare commit: the last commit at which the pair was current.\n"
            if unreleased else "")
    return f"""# Generated by tools/generate_raw_msgs.py. Do not edit.
#
# Builds gpsd_client against libgps at the revision this project generates
# {message} from, and runs the full test suite against it.
{note}#
# All the logic is in the reusable gpsd_api_shared.yml; this file exists so
# API {major}.{minor} has its own name, badge and re-run button in the Actions list.
name: gpsd API {major}.{minor}

on:
  push:
    branches: [{", ".join(PUSH_BRANCHES)}]
  pull_request:
    paths:
      # Only the things that can change what this version builds or publishes.
      # A docs-only change should not rebuild GPSd from source.
      - 'gps_extended_msgs/**'
      - 'gpsd_client/**'
      - 'tools/generate_raw_msgs.py'
      - 'tools/test_against_gpsd.sh'
      - '.github/workflows/gpsd_api_shared.yml'
      - '.github/workflows/gpsd_api_{major}v{minor}.yml'
  workflow_dispatch:

jobs:
  test:
    uses: ./.github/workflows/gpsd_api_shared.yml
    with:
      gpsd: '{rev.replace("release-", "")}'
      api: '{major}.{minor}'
      msg: '{message}'
"""


def generate(repo: str, scope: str) -> Dict[str, str]:
    """Return {relative path: contents} for every generated file."""
    if scope not in SCOPES:
        raise SystemExit(f"unknown scope {scope!r}; expected one of "
                         f"{list(SCOPE_ORDER)}")
    scope_members: List[str] = []
    for key in SCOPE_ORDER:
        scope_members += list(SCOPES[key])
        if key == scope:
            break

    files: Dict[str, str] = {
        "gpsd_client/include/gpsd_client/parsers/generated/gpsd_has_member.hpp":
            HAS_MEMBER_HEADER,
        "gpsd_client/include/gpsd_client/gpsd_raw_message.hpp":
            emit_selection_ladder(),
        os.path.join(PACKAGE, "msg", "GPSDJson.msg"): JSON_MESSAGE,
    }

    for pair, rev in sorted(REFERENCE_REVS.items()):
        files[f".github/workflows/gpsd_api_{pair[0]}v{pair[1]}.yml"] = \
            emit_version_workflow(pair, rev)

    for pair in sorted(REFERENCE_REVS):
        rev = REFERENCE_REVS[pair]
        src = strip_comments(read_gps_h(repo, rev))

        found = (int(re.search(r"define GPSD_API_MAJOR_VERSION\s+(\d+)", src).group(1)),
                 int(re.search(r"define GPSD_API_MINOR_VERSION\s+(\d+)", src).group(1)))
        if found != pair:
            raise SystemExit(f"{rev} reports API {found[0]}.{found[1]}, "
                             f"expected {pair[0]}.{pair[1]}")
        maxchannels = int(re.search(r"^#define MAXCHANNELS\s+(\d+)", src, re.M).group(1))
        if maxchannels != EXPECTED_MAXCHANNELS[pair]:
            raise SystemExit(f"{rev}: MAXCHANNELS is {maxchannels}, manifest "
                             f"says {EXPECTED_MAXCHANNELS[pair]}")

        model = build_model(pair, src, scope_members)
        flatten_model(model)
        constants = mask_constants(src)
        assert_no_macro_collisions(constants, src, rev)
        root = versioned("GPSDRaw", pair)
        for name in model.order:
            files[f"{PACKAGE}/msg/{name}.msg"] = emit_msg(
                model, name, rev, constants if name == root else ())
        major, minor = pair
        files[f"gpsd_client/include/gpsd_client/parsers/generated/"
              f"gpsd_raw_fill_{major}v{minor}.hpp"] = emit_parser(
                  model, rev, constants)
    return files


def orphans(files: Dict[str, str], output_root: str) -> List[str]:
    """Checked-in files that look generated but are no longer produced.

    Scoped to the directories this generator owns and to its own naming, so it
    can never propose deleting a hand-written file. `msg/GPSD*.msg` is
    exactly what the message package globs, which is what makes a leftover
    dangerous rather than merely untidy.
    """
    owned = {
        # Every message in this package is generated -- gps_msgs keeps the
        # hand-written GPSFix/GPSStatus -- so ownership is the whole directory
        # rather than a name prefix. Keying on the prefix would strand the old
        # files the moment the prefix itself changed, which is precisely when
        # the glob would pick up two definitions of the same message.
        os.path.join(PACKAGE, "msg"): lambda n: n.endswith(".msg"),
        os.path.join("gpsd_client", "include", "gpsd_client", "parsers",
                     "generated"): lambda n: n.endswith(".hpp"),
        os.path.join(".github", "workflows"): lambda n: (
            n.startswith("gpsd_api_") and n[len("gpsd_api_"):-len(".yml")]
            .replace("v", "").isdigit()),
    }
    expected = set(files)
    found: List[str] = []
    for directory, belongs in owned.items():
        full_dir = os.path.join(output_root, directory)
        if not os.path.isdir(full_dir):
            continue
        for name in sorted(os.listdir(full_dir)):
            rel = os.path.join(directory, name)
            if belongs(name) and rel not in expected:
                found.append(rel)
    return found


def main(argv: Optional[Sequence[str]] = None) -> int:
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_repo = os.path.join(
        os.path.dirname(os.path.dirname(here)), ".gpsd_versions", "gpsd")

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--gpsd-repo", default=default_repo,
                        help="GPSd git clone to read gps.h from "
                             f"(default: {default_repo})")
    parser.add_argument("--scope", default=CHECKED_IN_SCOPE,
                        choices=list(SCOPE_ORDER),
                        help="widest publishing scope to emit "
                             f"(default: {CHECKED_IN_SCOPE}, what the tree holds)")
    parser.add_argument("--output-root", default=here,
                        help="repository root to write into")
    parser.add_argument("--check", action="store_true",
                        help="exit non-zero if the checked-in files differ "
                             "from a fresh run; writes nothing")
    parser.add_argument("--list", action="store_true",
                        help="list the files that would be written")
    args = parser.parse_args(argv)

    if not os.path.isdir(os.path.join(args.gpsd_repo, ".git")):
        raise SystemExit(f"not a git clone: {args.gpsd_repo}")

    files = generate(args.gpsd_repo, args.scope)

    if args.list:
        for path in sorted(files):
            print(path)
        return 0

    if args.check:
        drift = []
        for path, contents in sorted(files.items()):
            full = os.path.join(args.output_root, path)
            if not os.path.exists(full):
                drift.append(f"missing: {path}")
            elif open(full).read() != contents:
                drift.append(f"differs: {path}")
        drift += [f"orphan:  {p}" for p in orphans(files, args.output_root)]
        if drift:
            print("generated files are out of date; rerun "
                  "tools/generate_raw_msgs.py", file=sys.stderr)
            for line in drift:
                print(f"  {line}", file=sys.stderr)
            return 1
        print(f"{len(files)} generated files up to date")
        return 0

    for path, contents in sorted(files.items()):
        full = os.path.join(args.output_root, path)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        with open(full, "w") as handle:
            handle.write(contents)

    # Remove files this generator previously produced but no longer does.
    # Without this a rename leaves the old file behind, and since the message
    # package globs its directory, the stale copy is still built -- which is
    # exactly how a renamed message once produced two conflicting definitions.
    stale = orphans(files, args.output_root)
    for path in stale:
        os.remove(os.path.join(args.output_root, path))

    print(f"wrote {len(files)} files under {args.output_root}"
          + (f", removed {len(stale)} stale" if stale else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
