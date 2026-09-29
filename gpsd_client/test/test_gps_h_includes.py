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

"""Check that nothing in gpsd_client includes gps.h except gpsd_client/gps.hpp."""

import os
import re
import unittest

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# gps.hpp is the one place gps.h is included. The other two include it on
# purpose, to check the message constants survive a file that includes gps.h
# before any gpsd_client header, as a downstream package might.
ALLOWED = {
    os.path.join('include', 'gpsd_client', 'gps.hpp'),
    os.path.join('test', 'test_gps_h.cpp'),
    os.path.join('test', 'test_gpsd_raw_include_order.cpp'),
}

INCLUDE = re.compile(r'^\s*#\s*include\s*[<"](gps\.h|libgpsmm\.h)[>"]', re.M)


class GpsHIncludes(unittest.TestCase):

    def test_only_gps_hpp_includes_gps_h(self):
        offenders = []
        for directory in ('include', 'src', 'test'):
            for root, _dirs, files in os.walk(os.path.join(PACKAGE, directory)):
                for name in files:
                    if not name.endswith(('.c', '.cc', '.cpp', '.h', '.hh', '.hpp')):
                        continue
                    path = os.path.join(root, name)
                    relative = os.path.relpath(path, PACKAGE)
                    with open(path, encoding='utf-8') as f:
                        if INCLUDE.search(f.read()) and relative not in ALLOWED:
                            offenders.append(relative)
        self.assertEqual(
            [], sorted(offenders),
            'include <gpsd_client/gps.hpp> instead of gps.h: its STATUS_* macros '
            'collide with the ROS message constants')

    def test_the_allowed_files_exist(self):
        # So that renaming one does not quietly widen the exemption.
        for relative in ALLOWED:
            self.assertTrue(os.path.exists(os.path.join(PACKAGE, relative)), relative)
