#!/usr/bin/env bash
#
# test_against_gpsd.sh -- build gpsd_client against multiple gpsd releases
# and run its unit tests linked against each one.
#
# gpsd_client supports gpsd API versions 9 and newer (gpsd >= 3.20); older
# releases are rejected by a compile-time #error, so they are not tested.
#
# For each gpsd release tag this script:
#   1. builds libgps/libgpsmm from source at that tag (cached),
#   2. builds gpsd_client against that exact library,
#   3. runs test_gpsd_parser with that library on LD_LIBRARY_PATH.
#
# This is what exercises every parser variant: e.g. gpsd 3.20 (API 9)
# compiles and tests GpsdParserV9, which is preprocessed away when building
# against newer headers.
#
# Usage (with a ROS 2 environment sourced):
#   tools/test_against_gpsd.sh              # every release >= 3.20
#   tools/test_against_gpsd.sh 3.20 3.25    # only these releases
#
# Environment overrides:
#   GPSD_REPO_URL    gpsd git remote (default: https://gitlab.com/gpsd/gpsd.git)
#   GPSD_TEST_CACHE  cache directory (default: <workspace>/.gpsd_versions)
#   GPSD_FULL_BUILD  also build the gpsd daemon and Python module, and run the
#                    gpsfake end-to-end tests, which skip themselves otherwise.
#                    Costs several minutes per version, so it is meant for one:
#                        GPSD_FULL_BUILD=1 tools/test_against_gpsd.sh 3.27.5
#                    Full builds cache under install/<ver>-full, separately
#                    from the libgps-only ones.
#
# Requirements: git, scons, colcon, a C/C++ toolchain, and network access
# for the initial gpsd clone. gpsd builds, libgps installs, and per-version
# colcon build dirs are cached, so reruns only rebuild gpsd_client.

set -uo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(dirname "${SCRIPT_DIR}")
# Assumes the standard layout <workspace>/src/<repo>/tools/<this script>.
WORKSPACE=$(cd "${REPO_DIR}/../.." && pwd)
CACHE=${GPSD_TEST_CACHE:-${WORKSPACE}/.gpsd_versions}
GPSD_REPO_URL=${GPSD_REPO_URL:-https://gitlab.com/gpsd/gpsd.git}
GPSD_SRC=${CACHE}/gpsd
LOGS=${CACHE}/logs
MIN_VERSION=3.20

log() { printf '\n=== %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

# Echo a captured log so failures are diagnosable on CI, where the log files
# themselves are thrown away with the runner.
dump_log() {
  local file=$1
  printf -- '--- %s\n' "${file}"
  if [ -f "${file}" ]; then
    tail -n "${DUMP_LINES:-200}" "${file}"
  else
    printf '(no such file -- the step never got far enough to write it)\n'
  fi
  printf -- '--- end %s\n' "${file}"
}

# True if $1 >= $2 in version order.
version_ge() { [ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -n1)" = "$2" ]; }

command -v git >/dev/null || die "git is required"
command -v scons >/dev/null || die "scons is required (apt install scons)"
command -v colcon >/dev/null || die "colcon is required"
[ -n "${AMENT_PREFIX_PATH:-}" ] || die "source your ROS 2 environment first"

mkdir -p "${CACHE}" "${LOGS}"
# Keep colcon from crawling the cache when run from the workspace root.
touch "${CACHE}/COLCON_IGNORE"

# Old gpsd releases (e.g. 3.20) import the 'imp' module, which was removed
# in Python 3.12; give scons a minimal importlib-based stand-in.
PYSHIM=${CACHE}/pyshim
mkdir -p "${PYSHIM}"
cat > "${PYSHIM}/imp.py" <<'EOF'
"""Minimal stand-in for the 'imp' module (removed in Python 3.12),
covering the calls old gpsd SConstructs make."""
import importlib.machinery
import importlib.util
import sys


def find_module(name, path=None):
    spec = importlib.machinery.PathFinder.find_spec(name, path)
    if spec is None:
        raise ImportError(name)
    return (None, spec.origin, ('', '', 5))


def load_source(name, pathname, file=None):
    loader = importlib.machinery.SourceFileLoader(name, pathname)
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    sys.modules[name] = module
    return module
EOF

if [ ! -d "${GPSD_SRC}/.git" ]; then
  log "Cloning gpsd from ${GPSD_REPO_URL}"
  git clone --quiet --filter=blob:none "${GPSD_REPO_URL}" "${GPSD_SRC}" \
    || die "failed to clone gpsd"
else
  # Pick up new release tags; tolerate being offline on reruns.
  git -C "${GPSD_SRC}" fetch --quiet --tags 2>/dev/null || true
fi

if [ $# -gt 0 ]; then
  VERSIONS="$*"
else
  VERSIONS=$(git -C "${GPSD_SRC}" tag --list 'release-*' \
    | sed 's/^release-//' \
    | grep -E '^[0-9]+\.[0-9]+(\.[0-9]+)?$' \
    | sort -V \
    | while read -r v; do version_ge "${v}" "${MIN_VERSION}" && echo "${v}"; done)
fi
[ -n "${VERSIONS}" ] || die "no gpsd versions selected"

# Build libgps/libgpsmm only; the daemon, clients, python bindings, and man
# pages are irrelevant to the parser tests and only add build fragility.
# Resolve a selector to a git revision. A plain version number means the
# release tag; anything else is passed through as a commit-ish, which is what
# the API pairs that shipped in no release (9.1, 10.1, 13.0) need.
resolve_rev() {
  case "$1" in
    [0-9]*.[0-9]*) echo "release-$1" ;;
    *)             echo "$1" ;;
  esac
}

# Where a version's gpsd install lives. Full builds get their own prefix: they
# contain strictly more than a libgps-only build, so sharing one directory
# would make the cache's contents depend on which mode happened to populate it
# first.
prefix_for() {
  if [ -n "${GPSD_FULL_BUILD:-}" ]; then
    echo "${CACHE}/install/$1-full"
  else
    echo "${CACHE}/install/$1"
  fi
}

build_gpsd() {
  local ver=$1
  local prefix
  prefix=$(prefix_for "${ver}")
  local build_log=${LOGS}/gpsd-${ver}.log
  local rev
  rev=$(resolve_rev "${ver}")
  if [ -f "${prefix}/include/gps.h" ]; then
    return 0
  fi
  # Normally libgps/libgpsmm only: the daemon, clients, Python bindings and man
  # pages are irrelevant to the parser tests and only add build fragility.
  #
  # GPSD_FULL_BUILD additionally builds the daemon and the Python module, which
  # is what the gpsfake end-to-end test needs -- gpsfake spawns the one and
  # imports the other, and refuses to run if their versions disagree. It costs
  # several minutes more, so it is opt-in and used for one version rather than
  # all ten.
  local scons_flags="gpsd=False gpsdclients=False python=False"
  if [ -n "${GPSD_FULL_BUILD:-}" ]; then
    scons_flags="gpsd=True gpsdclients=True python=True"
  fi
  git -C "${GPSD_SRC}" checkout --quiet "${rev}" || return 1
  git -C "${GPSD_SRC}" clean -xdfq
  # shellcheck disable=SC2086  # scons_flags is a deliberate word list
  (cd "${GPSD_SRC}" &&
   PYTHONPATH="${PYSHIM}${PYTHONPATH:+:${PYTHONPATH}}" \
   scons -j"$(nproc)" prefix="${prefix}" shared=True \
         ${scons_flags} qt=False manbuild=False \
         install >"${build_log}" 2>&1) || return 1
}

libgps_dir() {
  local prefix=$1 d
  for d in lib lib64 "lib/$(gcc -dumpmachine 2>/dev/null)"; do
    if [ -e "${prefix}/${d}/libgps.so" ]; then
      echo "${prefix}/${d}"
      return 0
    fi
  done
  return 1
}

# The full API pair. The minor version matters now that the raw messages are
# named GPSDRaw<MAJOR>v<MINOR> -- 9.0 and 9.1 select different message types.
api_version() {
  awk '$2 == "GPSD_API_MAJOR_VERSION" { maj = $3 }
       $2 == "GPSD_API_MINOR_VERSION" { min = $3 }
       END { print maj "." min }' "$1/include/gps.h"
}

parser_for_api() {
  local major=${1%%.*}
  if [ "${major}" -le 9 ]; then
    echo GpsdParserV9
  else
    echo GpsdParserV16
  fi
}

# The raw message this API pair selects, per gpsd_raw_message.hpp.
raw_message_for_api() {
  echo "GPSDRaw${1%%.*}v${1##*.}"
}

# Returns 0 on pass; 1 = gpsd build failed, 2 = client build failed,
# 3 = tests failed, 4 = the test binary was never produced.
build_and_test_client() {
  local ver=$1
  local prefix
  prefix=$(prefix_for "${ver}")
  local base=${CACHE}/colcon/${ver}
  local libdir
  libdir=$(libgps_dir "${prefix}") || return 2

  # Tier 2 (gpsfake) skips itself unless it is told where to find a daemon and
  # the log corpus. Exported only for a full build, so the ordinary ten-version
  # runs stay exactly as they were.
  if [ -n "${GPSD_FULL_BUILD:-}" ]; then
    export GPSD_TIER2_PREFIX="${prefix}"
    export GPSD_REPO="${GPSD_SRC}"
    # scons installs the gps module outside prefix, into the interpreter's
    # site-packages. Put it ahead of anything else so a system-packaged gpsfake
    # of a different version cannot win -- gpsfake aborts on a version mismatch
    # with the daemon, and this is the likeliest way to cause one.
    local py_site
    py_site="/usr/local/lib/python$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')/dist-packages"
    if [ -d "${py_site}" ]; then
      export PYTHONPATH="${py_site}${PYTHONPATH:+:${PYTHONPATH}}"
    fi
  fi

  # libgps_INCLUDE_DIRS/libgps_LIBRARIES are the cache variables that
  # gpsd_client's CMakeLists otherwise fills via find_path/find_library;
  # presetting them pins the build to this exact gpsd install.
  (cd "${WORKSPACE}" &&
   colcon build --packages-up-to gpsd_client \
     --build-base "${base}/build" --install-base "${base}/install" \
     --cmake-args "-Dlibgps_INCLUDE_DIRS=${prefix}/include" \
                  "-Dlibgps_LIBRARIES=${libdir}/libgps.so" \
     >"${LOGS}/client-${ver}.log" 2>&1) || return 2

  # A missing binary would otherwise surface as an indistinguishable "tests
  # failed" (exit 127), so call it out separately.
  [ -x "${base}/build/gpsd_client/test_gpsd_parser" ] || return 4

  # Run through colcon rather than invoking the binaries directly. The gtest
  # targets pass APPEND_LIBRARY_DIRS for this version's libgps, so colcon can
  # find it without the LD_LIBRARY_PATH juggling this script used to do -- and
  # running them the same way a user would is the point.
  (cd "${WORKSPACE}" &&
   colcon test --packages-up-to gpsd_client \
     --build-base "${base}/build" --install-base "${base}/install" \
     >"${LOGS}/test-${ver}.log" 2>&1) || true

  (cd "${WORKSPACE}" &&
   colcon test-result --test-result-base "${base}/build" \
     >>"${LOGS}/test-${ver}.log" 2>&1) || return 3
}

RESULTS=""
FAILED=0

for ver in ${VERSIONS}; do
  log "gpsd ${ver}: building libgps"
  if ! build_gpsd "${ver}"; then
    echo "gpsd ${ver}: libgps build FAILED"
    dump_log "${LOGS}/gpsd-${ver}.log"
    RESULTS="${RESULTS}${ver}|?|?|FAIL(gpsd build)\n"
    FAILED=1
    continue
  fi

  prefix=$(prefix_for "${ver}")
  api=$(api_version "${prefix}")
  parser=$(parser_for_api "${api}")

  raw=$(raw_message_for_api "${api}")
  log "gpsd ${ver} (API ${api}, ${parser}, ${raw}): building gpsd_client and testing"
  build_and_test_client "${ver}"
  case $? in
    0) result="PASS" ;;
    2) result="FAIL(client build)"; FAILED=1 ;;
    3) result="FAIL(tests)"; FAILED=1 ;;
    4) result="FAIL(no test binary)"; FAILED=1 ;;
    *) result="FAIL"; FAILED=1 ;;
  esac
  echo "gpsd ${ver}: ${result}"
  case "${result}" in
    PASS) ;;
    "FAIL(tests)")
      dump_log "${LOGS}/test-${ver}.log"
      ;;
    *)
      # Both the client build log and the test log are relevant: the build may
      # have succeeded loudly and still not emitted the binary.
      dump_log "${LOGS}/client-${ver}.log"
      ;;
  esac
  RESULTS="${RESULTS}${ver}|${api}|${raw}|${result}\n"
done

log "Summary"
printf '%-12s %-6s %-16s %s\n' "GPSD" "API" "RAW MESSAGE" "RESULT"
printf '%b' "${RESULTS}" | while IFS='|' read -r ver api parser result; do
  [ -n "${ver}" ] && printf '%-12s %-6s %-16s %s\n' "${ver}" "${api}" "${parser}" "${result}"
done

exit "${FAILED}"
