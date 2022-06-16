# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch a talker and a listener in a component container."""

import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import ament_index_python.packages

import yaml

gpsd_client_share_dir = ament_index_python.packages.get_package_share_directory('gpsd_client')
gpsd_client_params_file = os.path.join(gpsd_client_share_dir, 'config', 'gpsd_client.yaml')
with open(gpsd_client_params_file, 'r') as f:
    gpsd_client_params = yaml.safe_load(f)['gpsd_client']['ros__parameters']

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='fix_and_odometry_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='gpsd_client',
                    plugin='gpsd_client::GPSDClientComponent',
                    name='gpsd_client',
                    parameters=[gpsd_client_params]),
                ComposableNode(
                    package='gps_tools',
                    plugin='gps_tools::UtmOdometryComponent',
                    name='utm_gpsfix_to_odometry_node')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
