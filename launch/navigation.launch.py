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

# Various modifications by Scott Horton for Elsabot robots

import os
import subprocess
import tempfile

from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

#change to the name of your own map here or specify map via param
MAP_NAME='upstairs2'
#MAP_NAME='backyard'
#MAP_NAME='backyard_simple'
#MAP_NAME='downstairs'

def generate_launch_description():

    rviz_config_path = os.path.join(get_package_share_directory('elsabot_4wd'), 'rviz', 'navigation.rviz')

    default_map_path = os.path.join(get_package_share_directory('elsabot_4wd'), 'maps', f'{MAP_NAME}.yaml')
    # Fix - merge keepout support from jeep
    default_keep_out_map_path = os.path.join(get_package_share_directory('elsabot_4wd'), 'maps', f'{MAP_NAME}_keep_out.yaml')

    # Nav2 parameter file and delta files
    nav2_config_path_base = os.path.join(get_package_share_directory('elsabot_4wd'), 'config', 'navigation.yaml')
    nav2_config_path_fixed_path_pure_pursuit_deltas = os.path.join(get_package_share_directory('elsabot_4wd'), 'config', 'navigation_fixed_path_pure_pursuit_deltas.yaml')
    nav2_config_path_gps_deltas = os.path.join(get_package_share_directory('elsabot_4wd'), 'config', 'navigation_gps_deltas.yaml')

    # Nav2 Behavior Trees to use based on the specified mode
    nav2_bt_through_poses_xml_path = os.path.join(get_package_share_directory('elsabot_4wd'), 'nav_bt', 'navigate_through_poses_w_replanning_and_recovery.xml')
    nav2_bt_to_pose_xml_path = os.path.join(get_package_share_directory('elsabot_4wd'), 'nav_bt', 'navigate_to_pose_w_replanning_and_recovery.xml')
    nav2_bt_follow_point_xml_path = os.path.join(get_package_share_directory('elsabot_4wd'), 'nav_bt', 'follow_point.xml')

    nav2_bringup_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    nav2_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    # Use an OpaqueFunction to prepare the nav config and select the behavior tree
    # suitable for the specified launch options.  The OpaqueFunction allows the launch options
    # to be used directly since they are available at this point.
    def prepare_nav_launch(context, *args, **kwargs):

        logger = logging.get_logger('launch.user')
        use_gps = context.launch_configurations['use_gps']
        nav2_beh_mode = context.launch_configurations['nav2_behavior_mode']

        logger.info('prepare_nav_launch: use_gps: {}, nav2_beh_mode: {}'.format(use_gps, nav2_beh_mode))

        # Combine base Nav yaml file with other deltas as needed
        nav_yaml = nav2_config_path_base
        yaml_to_merge = ""

        # Fix - support gps and fixed-path mode => base + gps changes + fixed-path mode changes
        if use_gps == 'True':
            yaml_to_merge = nav2_config_path_gps_deltas

        elif nav2_beh_mode == 'nav_fixed_path_with_pure_pursuit':
            yaml_to_merge = nav2_config_path_fixed_path_pure_pursuit_deltas

        if yaml_to_merge:
            logger.info("Merging nav config {}". format(yaml_to_merge))
            nav_yaml_tmp = tempfile.NamedTemporaryFile(mode='w', delete=False)
            process = subprocess.run("yq eval-all '. as $item ireduce ({}; . * $item)' " +
                                     nav2_config_path_base + " " + yaml_to_merge +
                                     ">" + nav_yaml_tmp.name,
                                      shell=True, capture_output=True)
            if process.returncode:
                 raise Exception("Failed to merge nav2 config for gps")

            nav_yaml = nav_yaml_tmp.name

        logger.debug("prepared nav yaml:\n")
        with open(nav_yaml, 'r') as f:
            logger.debug(f.read())
       
        # Determine the Nav2 BT tree to use based on the specified nav mode.
        nav_through_poses_bt_xml = nav2_bt_through_poses_xml_path

        if nav2_beh_mode == 'nav_follow':
            nav_to_pose_bt_xml = nav2_bt_follow_point_xml_path
        else:
            nav_to_pose_bt_xml = nav2_bt_to_pose_xml_path

        param_substitutions = {
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml
        }

        logger.debug("Using BTs: {}".format(str(param_substitutions)))

        # Re-write the nav parameters file to use our modified nav bt xml files.
        rewritten_yaml = RewrittenYaml(
            source_file=nav_yaml,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
            
        descriptions = [
            # Use the full nav2 bringup launch script when using AMCL with a MAP.
            # The bringup script launches localization and then the base nav2 ndoes.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch_path),
                launch_arguments={
                    'map': LaunchConfiguration("map"),
                    'use_sim_time': LaunchConfiguration("use_sim_time"),
                    'slam': LaunchConfiguration("slam"),
                    'params_file': rewritten_yaml,
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
                    'params_file': rewritten_yaml,
                    'autostart': 'True',
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    #'container_name': 'nav2_container'
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_gps"))
            ),
        ]
        return descriptions;

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='slam',
            default_value='False',
            description='Run SLAM to create a map'
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

        DeclareLaunchArgument(
            name='nav2_behavior_mode', 
            default_value='nav_to_pose',
            description='Navigation behavior mode to use'
        ),

        SetParameter(name='use_sim_time', value=LaunchConfiguration("use_sim_time")),

        OpaqueFunction(function=prepare_nav_launch),

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
