# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Stop the installed fix_translator with SIGINT and SIGTERM, and check it exits cleanly."""

# rclpy's signal handlers shut the context down under spin(), which used to
# escape as an RCLError traceback and exit status 1. The race hit most runs,
# so each signal is tried several times.

import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import pytest

FIX_TRANSLATOR = os.path.join(
    get_package_prefix('gps_tools'), 'lib', 'gps_tools', 'fix_translator')
RUNS = 5


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_exits_cleanly_on_signal(signum):
    for run in range(RUNS):
        proc = subprocess.Popen(
            [FIX_TRANSLATOR], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # Long enough for rclpy to be spinning, which is where the race was.
        time.sleep(1.5)
        proc.send_signal(signum)
        try:
            _, stderr = proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            pytest.fail(f'run {run}: fix_translator did not exit after {signum.name}')
        assert proc.returncode == 0, f'run {run}: exit {proc.returncode}\n{stderr}'
        assert 'Traceback' not in stderr, f'run {run}:\n{stderr}'
