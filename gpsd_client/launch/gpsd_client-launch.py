"""Launch a talker and a listener in a component container."""

import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import ament_index_python.packages

import yaml

gpsd_client_share_dir = ament_index_python.packages.get_package_share_directory('gpsd_client')
gpsd_client_params_file = os.path.join(gpsd_client_share_dir, 'config', 'gpsd_client.yaml')
with open(gpsd_client_params_file, 'r') as f:
    gpsd_client_params = yaml.safe_load(f)['gpsd_client']['ros__parameters']


def gpsd_client_node(plugin):
    """Describe the gpsd_client component, managed or not."""
    return ComposableNode(
        package='gpsd_client',
        plugin=plugin,
        name='gpsd_client',
        parameters=[gpsd_client_params])


def generate_launch_description():
    """Generate launch description with multiple components."""
    use_lifecycle = LaunchConfiguration('use_lifecycle')

    # The two clients differ only in when they connect and publish, so the
    # container is described twice rather than the node: ComposableNode
    # descriptions are built when the launch file is evaluated, before the
    # argument has a value to branch on.
    def container(plugin, condition):
        return ComposableNodeContainer(
            name='fix_and_odometry_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                gpsd_client_node(plugin),
                ComposableNode(
                    package='gps_tools',
                    plugin='gps_tools::UtmOdometryComponent',
                    name='utm_gpsfix_to_odometry_node')
            ],
            output='screen',
            condition=condition,
        )

    unmanaged = container('gpsd_client::GPSDClientComponent',
                          UnlessCondition(use_lifecycle))
    managed = container('gpsd_client::GPSDClientLifecycleComponent',
                        IfCondition(use_lifecycle))

    def shutdown_on_exit(target):
        return launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=target,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())]
                ))

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_lifecycle',
            default_value='false',
            description='Run the managed (lifecycle) client instead of the '
                        'unmanaged one. The managed node starts unconfigured '
                        'and publishes nothing until it is configured and '
                        'activated, e.g. with `ros2 lifecycle set '
                        '/gpsd_client configure` then `... activate`.'),
        unmanaged,
        managed,
        shutdown_on_exit(unmanaged),
        shutdown_on_exit(managed),
    ])
