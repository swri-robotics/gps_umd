#!/usr/bin/env python3
"""End-to-end tests: gpsfake -> real GPSd -> gpsd_client -> ROS topics.

The other suites in this package stop at libgps: they hand JSON to
``gps_unpack()`` and inspect the resulting ``gps_data_t``. That covers parsing
thoroughly and runs everywhere, but it never starts a daemon, never opens a
socket and never publishes a message, so it cannot catch a fault in the parts
between those -- a publisher that is never advertised, a parameter that is not
read, a message type that fails to resolve at runtime.

This suite closes that gap the way GPSd's own test suite does. ``gpsfake``
replays a recorded receiver log into a real ``GPSd``; the node connects over
TCP through ``gpsmm`` exactly as it does in production; and the assertions are
made against messages that actually arrived on actual topics.

The ground truth is GPSd's, not ours. Every log in ``test/daemon`` ships with a
``.log.chk`` holding the JSON GPSd is expected to emit for it, so a published
position can be checked against what GPSd says that log means. A disagreement
means one of the two is wrong, which is the whole point.

Not run by default -- see the skip conditions below. Two things must be true,
and neither holds on the ROS build farm:

  * a GPSd built with the daemon *and* the Python module (``gpsd=True
    python=True``). ``tools/test_against_gpsd.sh`` deliberately builds neither,
    since every other suite needs only libgps.
  * GPSd's log corpus, i.e. a source checkout.

Point ``GPSD_E2E_PREFIX`` at the install prefix of such a build and
``GPSD_REPO`` at the matching source clone::

    GPSD_E2E_PREFIX=/opt/gpsd-3.27.5 GPSD_REPO=/src/gpsd \\
        colcon test --packages-select gpsd_client

Note "matching": gpsfake asserts its Python module and the daemon are the same
version and exits if they differ, so a system gpsfake cannot drive a
source-built daemon. The skip logic below checks this rather than letting it
surface as a confusing mid-test failure.
"""

import json
import os
import re
import socket
import subprocess
import sys
import time
import unittest

# --------------------------------------------------------------------------
# Prerequisites. All of this runs at import so the skip reason is specific --
# "no daemon at ..." is actionable, "ImportError" is not.
# --------------------------------------------------------------------------

E2E_PREFIX = os.environ.get("GPSD_E2E_PREFIX", "")
GPSD_REPO = os.environ.get("GPSD_REPO", "")
LOG_DIR = os.path.join(GPSD_REPO, "test", "daemon") if GPSD_REPO else ""

# What the C++ side selected, passed in by CMake, which is the only place that
# knows which gps.h this package compiled against. Without it the "the right
# message was selected" assertion would have nothing to compare to.
EXPECTED_RAW_MSG = os.environ.get("GPSD_EXPECTED_RAW_MSG", "")


def _find(*relative):
    """First existing path under the GPSd prefix, or ''."""
    for rel in relative:
        candidate = os.path.join(E2E_PREFIX, rel)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return ""


DAEMON = _find("sbin/gpsd", "bin/gpsd") if E2E_PREFIX else ""
GPSFAKE = _find("bin/gpsfake") if E2E_PREFIX else ""


def _python_module_dir():
    """Where the GPSd build put the 'gps' Python module.

    Resolve it by import rather than by path, but require the one belonging
    to this build rather than a system gpsfake of another version.
    """
    try:
        import gps  # noqa: F401
        import gps.fake  # noqa: F401
    except ImportError:
        return None, None
    return gps.__version__, os.path.dirname(gps.__file__)


GPS_PY_VERSION, GPS_PY_DIR = _python_module_dir()


def _daemon_version():
    if not DAEMON:
        return None
    try:
        out = subprocess.run([DAEMON, "-V"], capture_output=True, text=True,
                             timeout=30)
    except (OSError, subprocess.SubprocessError):
        return None
    match = re.search(r"([0-9]+\.[0-9]+(?:\.[0-9]+)?)", out.stdout + out.stderr)
    return match.group(1) if match else None


DAEMON_VERSION = _daemon_version()


def skip_reason():
    """Why this suite cannot run here, or None if it can."""
    if not E2E_PREFIX:
        return ("GPSD_E2E_PREFIX is unset -- needs a GPSd built with "
                "gpsd=True python=True")
    if not DAEMON:
        return f"no GPSd daemon under {E2E_PREFIX} (sbin/gpsd, bin/gpsd)"
    if not GPSFAKE:
        return f"no gpsfake under {E2E_PREFIX}/bin"
    if GPS_PY_VERSION is None:
        return "the 'gps' Python module is not importable"
    if not GPSD_REPO:
        return "GPSD_REPO is unset -- needs GPSd's test/daemon log corpus"
    if not os.path.isdir(LOG_DIR):
        return f"no log corpus at {LOG_DIR}"
    if DAEMON_VERSION and GPS_PY_VERSION != DAEMON_VERSION:
        # gpsfake exits on this itself; catching it here says which two things
        # disagree instead of failing inside a subprocess.
        return (f"version mismatch: gps module {GPS_PY_VERSION} from "
                f"{GPS_PY_DIR}, daemon {DAEMON_VERSION} at {DAEMON}. "
                "gpsfake requires them to match.")
    if not EXPECTED_RAW_MSG:
        return "GPSD_EXPECTED_RAW_MSG is unset -- CMake should set it"
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return "rclpy is not importable"
    return None


SKIP = skip_reason()

# --------------------------------------------------------------------------
# Ground truth
# --------------------------------------------------------------------------


def chk_reports(log_name):
    """The JSON reports GPSd is expected to emit for a log, by class.

    A .chk file interleaves the raw sentences with the JSON, so the JSON lines
    are picked out rather than the file being parsed as a whole.
    """
    path = os.path.join(LOG_DIR, log_name + ".chk")
    by_class = {}
    with open(path, errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                report = json.loads(line)
            except ValueError:
                continue
            by_class.setdefault(report.get("class"), []).append(report)
    return by_class


# 6 decimal places is ~10cm: far tighter than any error that matters here, and
# loose enough to absorb the rounding in the .chk's printed form. It is chosen
# to catch the failures that are actually plausible -- swapped lat/lon, degrees
# vs. radians, a field copied from its neighbour -- not to audit GPSd's math.
PLACES = 6


def rounded(value):
    return round(value, PLACES)


# --------------------------------------------------------------------------
# Session management
# --------------------------------------------------------------------------


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for_port(port, deadline):
    while time.time() < deadline:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1):
                return True
        except OSError:
            time.sleep(0.2)
    return False


class Session:
    """A gpsfake daemon plus a gpsd_client node, and the messages collected.

    One instance per log file. Starting these costs several seconds apiece, so
    a session is created once per log and every assertion about that log reads
    the same capture -- see CAPTURES.
    """

    # Seconds between replayed sentences, and the node's publish rate.
    #
    # These two are chosen together, and the relationship matters. The node
    # drains every queued report each tick and publishes from the last one,
    # while libgps assigns `set` fresh on every read -- so when more than one
    # report lands in a tick, all but the last are discarded. Data carried in
    # gps_data_t's union (GST, RTCM, SUBFRAME, ...) is lost outright, because
    # the next report clears both the bit and the arm.
    #
    # Replaying slower than the node polls gives each report its own tick,
    # which is what makes the union-carried classes observable at all. It is
    # also the realistic case: receivers emit at 1-10 Hz, not 50.
    CYCLE = 0.15      # ~6.7 reports/sec
    PUBLISH_RATE = 10  # Hz

    def __init__(self, log_name, rtcm=False, seconds=12.0, cycle=None):
        self.log_name = log_name
        self.rtcm = rtcm
        self.seconds = seconds
        if cycle is not None:
            self.CYCLE = cycle
        self.port = free_port()
        self.gpsfake = None
        self.node = None
        self.messages = {}
        self.topic_types = {}

    # -- process lifecycle -------------------------------------------------

    def _env(self):
        env = os.environ.copy()
        # gps.fake locates the daemon through GPSD_HOME before falling back to
        # PATH, so this pins it to the build under test rather than any system onee.
        env["GPSD_HOME"] = os.path.dirname(DAEMON)
        env["PATH"] = os.path.dirname(GPSFAKE) + os.pathsep + env.get("PATH", "")
        lib = os.path.join(E2E_PREFIX, "lib")
        if os.path.isdir(lib):
            env["LD_LIBRARY_PATH"] = lib + os.pathsep + env.get("LD_LIBRARY_PATH", "")
        return env

    def start(self):
        log_path = os.path.join(LOG_DIR, self.log_name)
        # -t   TCP rather than a pty: pty allocation is the one part of gpsfake
        #      that fails in a container, and nothing here needs a tty.
        # -n   feed the daemon without waiting for a client to connect first,
        #      so the node sees data as soon as it subscribes.
        # -c   slow the replay down. Left alone, gpsfake pushes the log through
        #      as fast as the CPU allows -- tens of thousands of reports a
        #      second, which tells us nothing and starves everything else.
        #      See CYCLE for why the specific value matters.
        # No -1, so the log cycles: the capture window can then be sized for
        # reliability rather than being bounded by the length of the log.
        self.gpsfake = subprocess.Popen(
            [GPSFAKE, "-t", "-P", str(self.port), "-n",
             "-c", str(self.CYCLE), log_path],
            env=self._env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        if not wait_for_port(self.port, time.time() + 60):
            self.stop()
            raise AssertionError(
                f"gpsfake never listened on port {self.port} for {self.log_name}")

        params = [
            "-p", "host:=127.0.0.1",
            "-p", f"port:={self.port}",
            "-p", "publish_gpsd_raw:=true",
            "-p", f"publish_gpsd_rtcm:={'true' if self.rtcm else 'false'}",
            "-p", f"publish_rate:={self.PUBLISH_RATE}",
            "-p", "frame_id:=gps",
            "-p", "use_gps_time:=false",
            "-p", "check_fix_by_variance:=false",
            "-p", "override_augmentation_source:=false",
        ]
        self.node = subprocess.Popen(
            ["ros2", "component", "standalone",
             "gpsd_client", "gpsd_client::GPSDClientComponent"] + params,
            env=self._env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)

    def stop(self):
        for proc in (self.node, self.gpsfake):
            if proc is None or proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), 15)
            except OSError:
                pass
        for proc in (self.node, self.gpsfake):
            if proc is None:
                continue
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), 9)
                except OSError:
                    pass

    # -- collection --------------------------------------------------------

    def collect(self):
        """Subscribe to the node's topics and gather messages for a while."""
        import importlib
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node

        # Its own context, and an executor bound to it. Several sessions are
        # created and torn down in one process, and rclpy's global context and
        # global executor do not survive that -- rclpy.spin_once() would reach
        # for the default context and fail on the second init.
        context = rclpy.Context()
        rclpy.init(context=context)
        try:
            listener = Node("gpsfake_listener", context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(listener)
            deadline = time.time() + self.seconds

            wanted = ["/fix", "/extended_fix", "/gpsd_raw"]
            if self.rtcm:
                wanted += ["/gpsd_rtcm2", "/gpsd_rtcm3"]

            # The raw topic's type is discovered rather than assumed: which
            # GPSDRaw<M>v<m> the node advertises is itself under test, so
            # hard-coding it here would assert nothing.
            subscribed = {}
            while time.time() < deadline:
                for name, types in listener.get_topic_names_and_types():
                    if name not in wanted or name in subscribed:
                        continue
                    self.topic_types[name] = types[0]
                    package, _, message = types[0].split("/")
                    cls = getattr(importlib.import_module(package + ".msg"), message)
                    self.messages.setdefault(name, [])
                    subscribed[name] = listener.create_subscription(
                        cls, name,
                        lambda msg, key=name: self.messages[key].append(msg), 10)
                executor.spin_once(timeout_sec=0.2)

            executor.remove_node(listener)
            listener.destroy_node()
        finally:
            rclpy.shutdown(context=context)
        return self.messages

    def diagnostics(self):
        """Subprocess output, for when an assertion fails and it is not obvious."""
        out = []
        for label, proc in (("gpsfake", self.gpsfake), ("node", self.node)):
            if proc is None:
                continue
            try:
                proc.stdout.flush()
            except (OSError, ValueError):
                pass
            out.append(f"--- {label} exit={proc.poll()}")
        return "\n".join(out)


# Sessions are expensive; each log is replayed once and every test that cares
# about that log reads the same capture.
CAPTURES = {}


def capture(log_name, rtcm=False, seconds=12.0, cycle=None):
    key = (log_name, rtcm, cycle)
    if key not in CAPTURES:
        session = Session(log_name, rtcm=rtcm, seconds=seconds, cycle=cycle)
        session.start()
        try:
            session.collect()
        finally:
            session.stop()
        CAPTURES[key] = session
    return CAPTURES[key]


@unittest.skipIf(SKIP, SKIP or "")
class EndToEnd(unittest.TestCase):
    """Assertions over a real replay of ac12.log (TPV + SKY)."""

    @classmethod
    def setUpClass(cls):
        cls.session = capture("ac12.log")
        cls.chk = chk_reports("ac12.log")

    def raws(self):
        return self.session.messages.get("/gpsd_raw", [])

    def test_all_three_topics_publish(self):
        # None of this is reachable from a library-only test:
        # because none of it exists until a node is running.
        for topic in ("/fix", "/extended_fix", "/gpsd_raw"):
            self.assertTrue(
                self.session.messages.get(topic),
                f"nothing published on {topic}\n{self.session.diagnostics()}")

    def test_raw_topic_advertises_the_message_for_this_api_version(self):
        # The selection ladder picks GPSDRaw<M>v<m> at compile time from the
        # gps.h being built against. A build that compiles but selects the
        # previous version's message would pass every other test in this
        # package; this is where that shows up.
        self.assertEqual(f"gps_extended_msgs/msg/{EXPECTED_RAW_MSG}",
                         self.session.topic_types.get("/gpsd_raw"))

    def test_published_positions_are_ones_gpsd_says_this_log_contains(self):
        truth = {(rounded(r["lat"]), rounded(r["lon"]))
                 for r in self.chk.get("TPV", []) if "lat" in r and "lon" in r}
        self.assertTrue(truth, "ac12.log.chk carries no TPV positions")

        seen = set()
        for msg in self.session.messages.get("/fix", []):
            # NaN latitude means no fix yet; nothing to check against.
            if msg.latitude != msg.latitude:
                continue
            seen.add((rounded(msg.latitude), rounded(msg.longitude)))
        self.assertTrue(seen, "no /fix message ever carried a position")

        # Every position published must be one GPSd derived from this log.
        # Catches a swapped lat/lon or a unit error, neither of which the
        # "did anything publish" test above would notice.
        self.assertEqual(set(), seen - truth,
                         "positions published that are not in the .chk ground truth")

    def test_skyview_length_always_agrees_with_the_count(self):
        # The array is trimmed to satellites_visible on the way out; a
        # mismatch means the trim is wrong and subscribers are reading
        # uninitialised entries.
        raws = self.raws()
        self.assertTrue(raws, "no raw messages captured")
        for msg in raws:
            self.assertEqual(len(msg.skyview), msg.satellites_visible)

    def test_satellite_counts_are_ones_gpsd_reports_for_this_log(self):
        truth = {len(r.get("satellites", [])) for r in self.chk.get("SKY", [])}
        truth.add(0)  # dop-only SKY reports clear the skyview; see the README
        seen = {msg.satellites_visible for msg in self.raws()}
        self.assertTrue(seen, "no raw messages captured")
        self.assertEqual(set(), seen - truth,
                         f"satellite counts not in the .chk: {sorted(seen - truth)}")

    def test_rtcm_is_absent_from_the_raw_message(self):
        # RTCM travels on its own topics. The unit tests assert the field
        # is gone from the message definition; this asserts the topics stay
        # unadvertised when publish_gpsd_rtcm is false.
        raws = self.raws()
        self.assertTrue(raws)
        fields = set(type(raws[0]).get_fields_and_field_types())
        self.assertNotIn("rtcm2", fields)
        self.assertNotIn("rtcm3", fields)
        self.assertNotIn("/gpsd_rtcm2", self.session.topic_types)
        self.assertNotIn("/gpsd_rtcm3", self.session.topic_types)

    def test_nan_survives_to_the_topic(self):
        # GPSd's "unknown" sentinel must not be flattened to 0.0 anywhere in
        # the chain -- including by message serialization, which no unit test
        # in this package exercises.
        raws = self.raws()
        unknowns = [m for m in raws if m.fix.eph != m.fix.eph or m.fix.epv != m.fix.epv]
        if not unknowns:
            self.skipTest("this replay never produced an unknown error estimate")
        self.assertTrue(unknowns)


@unittest.skipIf(SKIP, SKIP or "")
class GstReports(unittest.TestCase):
    """gr8013-w.log carries GST, which reaches the `gst` member."""

    @classmethod
    def setUpClass(cls):
        cls.session = capture("gr8013-w.log")
        cls.chk = chk_reports("gr8013-w.log")

    def test_gst_values_reach_the_message(self):
        # The JSON key is "lat"; the struct member -- and so the message field
        # -- is lat_err_deviation. They are the same quantity under two names,
        # which is exactly the kind of mismatch this test is here to catch.
        truth = {rounded(r["lat"]) for r in self.chk.get("GST", []) if "lat" in r}
        self.assertTrue(truth, "gr8013-w.log.chk carries no GST latitude error")
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        seen = {rounded(m.gst.lat_err_deviation) for m in raws
                if m.gst.lat_err_deviation == m.gst.lat_err_deviation}
        if not seen:
            self.skipTest("this replay sampled no GST report")
        self.assertEqual(set(), seen - truth,
                         "gst.lat_err_deviation values GPSd does not report")


@unittest.skipIf(SKIP, SKIP or "")
class AttitudeReports(unittest.TestCase):
    """tnt-revolution.log carries ATT, which reaches the `attitude` member.

    The log choice matters. tnt-revolution is a dedicated heading sensor: 60 of
    its 120 reports are ATT, the first arrives at report 2, and the whole log
    replays inside one capture window. Zero ATT therefore means a real failure
    rather than a sampling accident, so this suite asserts instead of skipping.

    A log whose ATT reports sit past the capture window would skip forever and
    report that as success, so prefer density near the start over total count.

    On the API this suite runs, `attitude` sits past the union's closing brace
    and UNION_SET omits ATTITUDE_SET, so no following report clobbers it and
    the CYCLE rationale on Session does not apply. attitude left the union at
    API 12.0, so on API 9-11 it is union-carried and CYCLE would matter; this
    suite runs one recent version and never meets that case.
    """

    # Every ATT report in this log carries all five, in disjoint ranges
    # (heading ~14000, dip ~13600, pitch ~170, roll ~-40). Crossing a field
    # with its neighbour therefore produces a value the .chk never reported
    # rather than a plausible number.
    FIELDS = ("heading", "pitch", "roll", "dip", "mag_x")

    @classmethod
    def setUpClass(cls):
        cls.session = capture("tnt-revolution.log")
        cls.chk = chk_reports("tnt-revolution.log")

    def test_attitude_values_reach_the_message(self):
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        reports = self.chk.get("ATT", [])
        self.assertTrue(reports, "tnt-revolution.log.chk carries no ATT report")

        for field in self.FIELDS:
            with self.subTest(field=field):
                truth = {rounded(r[field]) for r in reports if field in r}
                self.assertTrue(truth, f"no ATT report carries {field}")
                # NaN is GPSd's "unset"; compare only what was actually filled.
                seen = {rounded(getattr(m.attitude, field)) for m in raws
                        if getattr(m.attitude, field) == getattr(m.attitude, field)}
                self.assertTrue(
                    seen,
                    f"no attitude.{field} reached the topic, though the log "
                    f"reports {len(truth)} distinct values\n"
                    f"{self.session.diagnostics()}")
                self.assertEqual(
                    set(), seen - truth,
                    f"attitude.{field} values published that GPSd does not report")

    def test_attitude_tracks_the_replay(self):
        """Every field must move, not just be present once.

        attitude persists across reads because it is not a union arm, so a fill
        that ran once would satisfy the subset check above forever. This
        separates "filled and held" from "tracked".

        The threshold takes half the distinct values GPSd reports, a 2x margin
        over full coverage. gpsfake replays without -1, so the log cycles, and
        one cycle (71 sentences x CYCLE, ~10.6s) fits inside the capture window
        with room to spare -- a full pass arrives whatever phase collection
        starts on.

        Subset above and coverage here stay separate on purpose. Exact set
        equality would read as stronger but would fail whenever a loaded runner
        drops one publish, and the two catch different bugs: wrong values
        above, frozen values here.
        """
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        reports = self.chk.get("ATT", [])
        for field in self.FIELDS:
            with self.subTest(field=field):
                truth = {rounded(r[field]) for r in reports if field in r}
                seen = {rounded(getattr(m.attitude, field)) for m in raws
                        if getattr(m.attitude, field) == getattr(m.attitude, field)}
                self.assertGreaterEqual(
                    len(seen), len(truth) // 2,
                    f"attitude.{field} barely moved across the replay: "
                    f"{len(seen)} distinct values published of {len(truth)} "
                    f"reported -- it is being filled once and then held, not "
                    f"tracked\n{sorted(seen)}")


@unittest.skipIf(SKIP, SKIP or "")
class SubframeAndLogAreUnreachableThroughLibgps(unittest.TestCase):
    """SUBFRAME and LOG never reach a socket client, and this pins that.

    The daemon emits both -- ublox-ned-m8t-sbfrx3 is 151 SUBFRAME reports and a
    plain JSON watcher receives them -- but ``libgps_json.c`` has no reader for
    either class. It decodes AIS, ATT, DEVICE, DEVICES, ERROR, GST, IMU, OSC,
    PPS, RAW, RTCM2, RTCM3, SKY, TOFF, TPV, VERSION and WATCH, and silently
    ignores everything else. So ``gps_data_t::subframe`` and ``::log`` are only
    ever populated inside GPSd itself, never in a client.

    That means the message fields exist and are correct, and in production will
    always be empty. Asserting the emptiness is worth more than deleting the
    tests: it is the difference between a known property of libgps and a bug in
    our fill code, and only an end-to-end test can tell those apart.

    If a future libgps learns to parse them these tests fail, which is exactly
    when someone should look at the subframe dispatch again.
    """

    @classmethod
    def setUpClass(cls):
        cls.session = capture("ublox-ned-m8t-sbfrx3.log")
        cls.chk = chk_reports("ublox-ned-m8t-sbfrx3.log")

    def test_the_daemon_really_does_emit_subframes_for_this_log(self):
        # Guards the premise. Without this, the assertions below would also
        # pass against a log that simply contains no subframes.
        self.assertTrue(self.chk.get("SUBFRAME"),
                        "picked a log with no SUBFRAME reports in its .chk")

    def test_subframe_arm_stays_empty(self):
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        populated = [m for m in raws if m.subframe]
        self.assertEqual(
            [], populated,
            "subframe was populated -- either libgps now parses SUBFRAME, or "
            "the union arm is being read when it is not the live one")
        self.assertFalse(any(m.set & type(m).SET_SUBFRAME for m in raws),
                         "SUBFRAME_SET appeared in the mask of a socket client")

    def test_log_member_stays_unset(self):
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        # gps_data_t::log is a plain member, not a union arm, so "unset" means
        # its NaN sentinel survived rather than the arm being absent.
        self.assertTrue(all(m.log.lat != m.log.lat for m in raws),
                        "log.lat carried a value; libgps has no LOG reader")


@unittest.skipIf(SKIP, SKIP or "")
class RawMeasurements(unittest.TestCase):
    """ublox-neo-m8t carries RAW pseudorange measurements."""

    @classmethod
    def setUpClass(cls):
        cls.session = capture("ublox-neo-m8t.log")
        cls.chk = chk_reports("ublox-neo-m8t.log")

    def test_raw_arm_carries_measurements(self):
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        seen = [m.raw[0] for m in raws if m.raw]
        if not seen:
            self.skipTest("this replay sampled no RAW report")
        # Every RAW report in the corpus carries at least one measurement, so
        # an empty meas[] means the arm was copied without its payload.
        self.assertTrue(any(len(r.meas) > 0 for r in seen),
                        "RAW arm published with no measurements in any sample")


@unittest.skipIf(SKIP, SKIP or "")
class OscillatorReports(unittest.TestCase):
    """isync is the corpus's only OSC log.

    The plan named ericsson-gru04 for this; there is no such log. isync is the
    only one that produces the class at all.
    """

    @classmethod
    def setUpClass(cls):
        cls.session = capture("isync.log")
        cls.chk = chk_reports("isync.log")

    def test_oscillator_arm_matches_the_chk(self):
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        seen = [m.osc[0] for m in raws if m.osc]
        if not seen:
            self.skipTest("this replay sampled no OSC report")
        truth = {r["delta"] for r in self.chk.get("OSC", []) if "delta" in r}
        self.assertTrue(truth)
        self.assertEqual(set(), {o.delta for o in seen} - truth,
                         "oscillator delta values GPSd does not report")


@unittest.skipIf(SKIP, SKIP or "")
class ImuReports(unittest.TestCase):
    """ublox-neo-m8u carries 890 IMU reports.

    imu[] is not a union arm and carries no count -- it is terminated by an
    empty attitude_t::msg. This is the end-to-end check on that trimming.
    """

    @classmethod
    def setUpClass(cls):
        cls.session = capture("ublox-neo-m8u.log")
        cls.chk = chk_reports("ublox-neo-m8u.log")

    def test_imu_entries_are_trimmed_and_labelled(self):
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        populated = [m for m in raws if m.imu]
        if not populated:
            self.skipTest("this replay sampled no IMU report")
        truth = {r["msg"] for r in self.chk.get("IMU", []) if "msg" in r}
        self.assertTrue(truth)
        for msg in populated:
            # The terminator rule: every published entry must have a non-empty
            # msg, or the trim ran past the end of the real data.
            for entry in msg.imu:
                self.assertTrue(entry.msg, "published an imu entry with no msg")
            self.assertEqual(set(), {e.msg for e in msg.imu} - truth)


@unittest.skipIf(SKIP, SKIP or "")
class RtcmTopics(unittest.TestCase):
    """ublox-zed-f9r.log carries RTCM3, which travels on its own topic."""

    @classmethod
    def setUpClass(cls):
        cls.session = capture("ublox-zed-f9r.log", rtcm=True, seconds=15.0)

    def test_rtcm3_topic_is_advertised_when_the_flag_is_set(self):
        self.assertIn("/gpsd_rtcm3", self.session.topic_types,
                      f"publish_gpsd_rtcm was true but the topic never "
                      f"appeared\n{self.session.diagnostics()}")

    def test_rtcm3_messages_arrive_and_carry_a_header(self):
        messages = self.session.messages.get("/gpsd_rtcm3", [])
        if not messages:
            self.skipTest("this replay sampled no RTCM3 report")
        self.assertTrue(messages[0].header.frame_id)

    def test_reports_are_not_republished(self):
        """Each RTCM report should reach the topic once, not once per cycle.

        This is a regression test for a specific defect. gps_data_t::set is not
        per-report for every class: GPSd's SKY handler ORs its bits in without
        clearing UNION_SET, so RTCM3_SET and the union arm behind it survive
        every SKY report until something later clears them. Publishing RTCM on
        the node's timer therefore republished whatever RTCM report came last,
        once per cycle, for as long as the stale bit lasted -- measured at 371
        messages for 97 distinct payloads.

        The fix is to publish per report and use the report's JSON class, which
        gps_read() hands back, rather than the mask. What that changes, and what
        this test watches for, is the rate of *consecutive identical* messages.

        A replay faster than the publish rate on purpose: the defect only
        appears when several reports arrive per cycle, so the slower default
        would hide it. Some genuine repetition is expected -- gpsfake loops the
        log, and RTCM streams resend the same corrections -- so this asserts a
        rate well below the broken behaviour (58%) and well above the healthy
        one (7%), rather than demanding zero.
        """
        session = capture("ublox-zed-f9r.log", rtcm=True, seconds=15.0, cycle=0.02)
        messages = session.messages.get("/gpsd_rtcm3", [])
        if len(messages) < 20:
            self.skipTest(f"only {len(messages)} RTCM3 messages; too few to judge")

        def body(msg):
            # Everything but the header: the stamp is assigned per publish, so
            # two publishes of one report differ only there.
            return tuple(str(getattr(msg, f))
                         for f in msg.get_fields_and_field_types() if f != "header")

        bodies = [body(m) for m in messages]
        repeats = sum(1 for a, b in zip(bodies, bodies[1:]) if a == b)
        ratio = repeats / len(bodies)
        self.assertLess(
            ratio, 0.25,
            f"{repeats} of {len(bodies)} RTCM3 messages repeated the previous "
            f"one ({ratio:.0%}); only {len(set(bodies))} distinct. The node is "
            "republishing a stale union arm rather than publishing per report.")

    def test_the_raw_topic_still_reports_that_rtcm_arrived(self):
        # The mask is the contract that lets a raw subscriber know an RTCM
        # report happened even though the payload went elsewhere.
        raws = self.session.messages.get("/gpsd_raw", [])
        self.assertTrue(raws, "no raw messages captured")
        bit = type(raws[0]).SET_RTCM3
        if not any(m.set & bit for m in raws):
            self.skipTest("this replay sampled no RTCM3 report")


if __name__ == "__main__":
    if SKIP:
        sys.stderr.write(f"skipped: {SKIP}\n")
    unittest.main()
