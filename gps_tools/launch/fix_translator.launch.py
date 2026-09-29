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

"""Run fix_translator, which translates between NavSatFix and GPSFix."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Each of fix_translator's topics, and the name it is remapped to by default.
# By default it translates NavSatFix on fix to GPSFix on gps_fix. The GPSFix to
# NavSatFix direction keeps its own names until they are set.
TOPICS = {
    'navsat_fix_in': 'fix',
    'gps_fix_out': 'gps_fix',
    'gps_fix_in': 'gps_fix_in',
    'navsat_fix_out': 'navsat_fix_out',
}


def generate_launch_description():
    """Launch fix_translator with a launch argument per topic."""
    arguments = [
        DeclareLaunchArgument(topic, default_value=default,
                              description=f'Topic remapped from {topic}')
        for topic, default in TOPICS.items()
    ]
    translator = Node(
        package='gps_tools',
        executable='fix_translator',
        name='fix_translator',
        remappings=[(topic, LaunchConfiguration(topic)) for topic in TOPICS],
        output='screen',
    )
    return launch.LaunchDescription(arguments + [translator])
