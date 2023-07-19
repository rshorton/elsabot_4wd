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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

MAP_NAME='upstairs2'
#MAP_NAME='backyard'
#MAP_NAME='downstairs'

def generate_launch_description():

    elsabot_4wd_dir = get_package_share_directory('elsabot_4wd')

    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'description.launch.py']
    )

    rosbridge_launch_path = PathJoinSubstitution(
        [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'rviz', 'navigation.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_gps', 
            default_value='False',
            description='Set to True to use GPS'
        ),

        DeclareLaunchArgument(
            name='use_gps_rtk', 
            default_value='False',
            description='Set to True to use GPS RTK mode'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=os.path.join(elsabot_4wd_dir, 'maps', f'{MAP_NAME}.yaml'),
            description='Full path to map yaml file to load'
        ),

        DeclareLaunchArgument(
            name='nav2_params_file',
            default_value=os.path.join(elsabot_4wd_dir, 'config', 'navigation.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            name='slam',
            default_value='False',
            description='Run SLAM to create a map'
        ),

        DeclareLaunchArgument(
            name='use_keep_out',
            default_value='False',
            description='Enable use of keep-out area map'
        ),

        DeclareLaunchArgument(
            name='use_nav',
            default_value='False',
            description='Enable also start nav2'
        ),

        DeclareLaunchArgument(
            name='base_serial_port', 
            default_value='/dev/teensy',
            description='Linorobot Base Serial Port'
        ),

        DeclareLaunchArgument(
            name='joy', 
            default_value='true',
            description='Use Joystick'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration('base_serial_port')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            condition=IfCondition(LaunchConfiguration('joy')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            condition=IfCondition(LaunchConfiguration('use_nav')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'map': LaunchConfiguration('map'),
                'slam': LaunchConfiguration('slam'),
                'params_file': LaunchConfiguration('nav2_params_file')
            }.items()
        ),

        # Conditionally start the map servers for supporting a map keep-out area
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'keep_out_area.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_keep_out')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'sensor_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_gps': LaunchConfiguration('use_gps'),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'gps.launch.py')),
            launch_arguments={
               'use_gps_rtk': LaunchConfiguration('use_gps_rtk')
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_gps'))
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # web bridge (for proxying topics/actions to/from ros_web based applications)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_path)
        )
    ])
