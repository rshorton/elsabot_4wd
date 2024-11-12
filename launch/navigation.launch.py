# Portions of this file are from the Elsabot project by Scott Horton
# and others from LinoRobot2 and Nav2.

# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, NotSubstitution, AndSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare

#change to the name of your own map here or specify map via param
MAP_NAME='upstairs_1111'
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

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_composition = LaunchConfiguration('use_composition')

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

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
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'yaml_filename': default_map_path
        }

        logger.debug("Using BTs: {}".format(str(param_substitutions)))

        # Re-write the nav parameters file to use our modified nav bt xml files.
        rewritten_yaml = RewrittenYaml(
            source_file=nav_yaml,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
            
        nav2_config_file = rewritten_yaml.perform(context)
        # Dump the config to check it
        logger.debug("complete nav yaml:\n")
        with open(nav2_config_file, 'r') as f:
            logger.info(f.read())

        descriptions = [
            GroupAction([
                PushRosNamespace(
                    condition=IfCondition(use_namespace),
                    namespace=namespace),

                Node(
                    condition=IfCondition(use_composition),
                    name='nav2_container',
                    package='rclcpp_components',
                    executable='component_container_isolated',
                    parameters=[nav2_config_file, {'autostart': autostart}],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                    output='screen'
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'slam_launch.py')),
                    condition=IfCondition(slam),
                    launch_arguments={'namespace': namespace,
                                    'use_sim_time': use_sim_time,
                                    'autostart': autostart,
                                    'use_respawn': use_respawn,
                                    'params_file': nav2_config_file}.items()
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
                    condition=IfCondition( AndSubstitution( NotSubstitution(slam), NotSubstitution(use_gps) ) ),
                    launch_arguments={'namespace': '',
                                    'map': default_map_path,
                                    'use_sim_time': use_sim_time,
                                    'autostart': autostart,
                                    'params_file': nav2_config_file,
                                    'use_composition': use_composition,
                                    'use_respawn': use_respawn,
                                    'container_name': 'nav2_container'}.items()
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
                    launch_arguments={'namespace': namespace,
                                    'use_sim_time': use_sim_time,
                                    'autostart': autostart,
                                    'params_file': nav2_config_file,
                                    'use_composition': use_composition,
                                    'use_respawn': use_respawn,
                                    'container_name': 'nav2_container'}.items()
                )
            ])
        ]
        HIDE_descriptions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration("use_sim_time"),
                    'map': default_map_path,
                    'slam': slam,
                    'log_level': "info",
                    'params_file': nav2_config_file
                }.items()
            )
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

        DeclareLaunchArgument(
            name='log_level',
            default_value='info',
            description='log level'
        ),
    
        DeclareLaunchArgument(
            name='autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            name='use_composition',
            default_value='True',
            description='Whether to use composed bringup'
        ),

        DeclareLaunchArgument(
            name='namespace',
            default_value='',
            description='Top-level namespace'
        ),

        DeclareLaunchArgument(
            name='use_namespace',
            default_value='False',
            description='Whether to apply a namespace to the navigation stack'
        ),

        DeclareLaunchArgument(
            name='use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.'
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
