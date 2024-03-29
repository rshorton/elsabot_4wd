# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Various modifications by Scott Horton for Elsabot robots

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    joy_config_path = PathJoinSubstitution(
        [FindPackageShare("elsabot_4wd"), "config", "joy.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='cmd_vel_topic', 
            default_value='cmd_vel',
            description='Topic to publish cmd_vel messages'
        ),        

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config_path],
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
        )
    ])