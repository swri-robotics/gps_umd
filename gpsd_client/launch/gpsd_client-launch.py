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

    return launch.LaunchDescription([container,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=container,
                    on_exit=[launch.actions.EmitEvent(
                        event=launch.events.Shutdown())]
                    ))
                ])

