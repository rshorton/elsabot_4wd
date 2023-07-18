# Portions of this file are from the Elsabot project by Scott Horton
# and others from LinoRobot2

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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

#change to the name of your own map here or specify map via param
MAP_NAME='upstairs2'
#MAP_NAME='backyard'
#MAP_NAME='backyard_simple'
#MAP_NAME='downstairs'

def generate_launch_description():
    nav2_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'rviz', 'navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'maps', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'config', 'navigation.yaml']
    )

    nav2_config_path_gps = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'config', 'navigation_gps.yaml']
    )

    use_gps = LaunchConfiguration('use_gps')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='use_gps', 
            default_value='False',
            description='Set to True to use GPS'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        # Tried using RewrittenYaml() to create a GPS mode param file from the full params,
        # but RewrittenYaml() is to primative in that it only compares the leaf nodes when
        # replacing.  See https://github.com/ros-planning/navigation2/issues/2117
        #
        # Tried to use yp to merge a base param file with just changes needed for GPS mode,
        # but this needs to block until the output file is created.
        # Shell=True is needed since ">" requires to save the merged changes.
        #ExecuteProcess(
        #    cmd=["yq", "eval-all", "'. as $item ireduce ({}; . * $item )'",
        #        PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'navigation.yaml']),
        #        PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'navigation_gps_changes.yaml']),
        #        ">", nav_params_file_for_gps
        #    ],
        #    log_cmd=True,
        #    shell=True,
        #    condition=IfCondition(LaunchConfiguration("use_gps"))
        #),

        # Use the full nav2 bringup launcher when using AMCL with a MAP.
        # The bringup script launches localization and then the base nav2 ndoes.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'params_file': nav2_config_path,
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("use_gps"))
        ),

        # Only launch the base nav2 nodes (without AMCL) when using GPS
        # FIX - change to use composition
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'params_file': nav2_config_path_gps,
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
                #'container_name': 'nav2_container'
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gps"))
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        )
    ])
