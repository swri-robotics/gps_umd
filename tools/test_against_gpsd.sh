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
build_gpsd() {
  local ver=$1
  local prefix=${CACHE}/install/${ver}
  local build_log=${LOGS}/gpsd-${ver}.log
  if [ -f "${prefix}/include/gps.h" ]; then
    return 0
  fi
  git -C "${GPSD_SRC}" checkout --quiet "release-${ver}" || return 1
  git -C "${GPSD_SRC}" clean -xdfq
  (cd "${GPSD_SRC}" &&
   PYTHONPATH="${PYSHIM}${PYTHONPATH:+:${PYTHONPATH}}" \
   scons -j"$(nproc)" prefix="${prefix}" shared=True \
         gpsd=False gpsdclients=False python=False qt=False manbuild=False \
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

api_version() {
  awk '$2 == "GPSD_API_MAJOR_VERSION" { print $3 }' "$1/include/gps.h"
}

parser_for_api() {
  if [ "$1" -le 9 ]; then
    echo GpsdParserV9
  else
    echo GpsdParserV16
  fi
}

# Returns 0 on pass; 1 = gpsd build failed, 2 = client build failed,
# 3 = tests failed, 4 = the test binary was never produced.
build_and_test_client() {
  local ver=$1
  local prefix=${CACHE}/install/${ver}
  local base=${CACHE}/colcon/${ver}
  local libdir
  libdir=$(libgps_dir "${prefix}") || return 2

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

  # colcon builds gps_msgs into this run's isolated install base, and nothing
  # else puts that on the library path. On a machine that already has gps_msgs
  # installed the loader quietly finds it in the underlay instead, so this is
  # only fatal where gps_msgs is *not* installed -- e.g. ros:*-ros-base on CI.
  local ws_libs=""
  for d in "${base}"/install/*/lib; do
    [ -d "${d}" ] && ws_libs="${ws_libs}${d}:"
  done

  # Prepend the freshly built libgpsd_client and this version's libgps so
  # neither can be shadowed by copies from an underlay on LD_LIBRARY_PATH.
  LD_LIBRARY_PATH="${base}/build/gpsd_client:${libdir}:${ws_libs}${LD_LIBRARY_PATH:-}" \
    "${base}/build/gpsd_client/test_gpsd_parser" \
    >"${LOGS}/test-${ver}.log" 2>&1 || return 3
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

  prefix=${CACHE}/install/${ver}
  api=$(api_version "${prefix}")
  parser=$(parser_for_api "${api}")

  log "gpsd ${ver} (API ${api}, ${parser}): building gpsd_client and testing"
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
  RESULTS="${RESULTS}${ver}|${api}|${parser}|${result}\n"
done

log "Summary"
printf '%-10s %-5s %-15s %s\n' "GPSD" "API" "PARSER" "RESULT"
printf '%b' "${RESULTS}" | while IFS='|' read -r ver api parser result; do
  [ -n "${ver}" ] && printf '%-10s %-5s %-15s %s\n' "${ver}" "${api}" "${parser}" "${result}"
done

exit "${FAILED}"
