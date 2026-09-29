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

"""Launch gpsd_client-launch.py, managed and not, and check what comes up."""

# Needs no GPSd. The unmanaged client logs that it cannot connect and stays
# loaded; the managed one waits, unconfigured, before it ever tries.

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from composition_interfaces.srv import ListNodes
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest
import rclpy

LAUNCH_FILE = os.path.join(
    get_package_share_directory('gpsd_client'), 'launch', 'gpsd_client-launch.py')
CONTAINER = '/fix_and_odometry_container'
COMPONENTS = {'/gpsd_client', '/utm_gpsfix_to_odometry_node'}


@pytest.mark.launch_test
@launch_testing.parametrize('use_lifecycle', ['false', 'true'])
def generate_test_description(use_lifecycle):
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LAUNCH_FILE),
            launch_arguments={'use_lifecycle': use_lifecycle}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class GpsdClientLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gpsd_client_launch')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, srv_type, name, request, timeout=30.0):
        client = self.node.create_client(srv_type, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=timeout),
                            f'{name} never appeared')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            self.assertTrue(future.done(), f'{name} never answered')
            return future.result()
        finally:
            self.node.destroy_client(client)

    def test_container_loads_both_components(self, use_lifecycle):
        # The container is up before launch has loaded anything into it.
        loaded = set()
        deadline = time.monotonic() + 30.0
        while not COMPONENTS <= loaded and time.monotonic() < deadline:
            response = self.call(ListNodes, CONTAINER + '/_container/list_nodes',
                                 ListNodes.Request())
            loaded = set(response.full_node_names)
            time.sleep(0.2)
        self.assertEqual(COMPONENTS, loaded)

    def test_loads_the_client_the_argument_asks_for(self, use_lifecycle):
        if use_lifecycle == 'true':
            response = self.call(GetState, '/gpsd_client/get_state', GetState.Request())
            self.assertEqual(State.PRIMARY_STATE_UNCONFIGURED, response.current_state.id)
        else:
            # Give the managed services time to show up if they were going to.
            time.sleep(2.0)
            services = {name for name, _ in self.node.get_service_names_and_types()}
            self.assertIn('/gpsd_client/get_parameters', services)
            self.assertNotIn('/gpsd_client/get_state', services)


@launch_testing.post_shutdown_test()
class GpsdClientLaunchShutdown(unittest.TestCase):

    def test_container_exits_cleanly(self, proc_info):
        # launch stops the container with SIGINT at the end of the test.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
