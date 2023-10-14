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
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

MAP_NAME='upstairs3'
#MAP_NAME='backyard'
#MAP_NAME='downstairs'

def generate_launch_description():

    elsabot_4wd_dir = get_package_share_directory('elsabot_4wd')

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'gazebo.launch.py']
    )

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

    # Load the cmd_vel_mux config since it expects the parameters vs the
    cmd_vel_mux_config_file = os.path.join(get_package_share_directory('elsabot_4wd'), 'config', 'cmd_vel_mux.yaml')
    with open(cmd_vel_mux_config_file, 'r') as f:
        cmd_vel_mux_config = yaml.safe_load(f)['cmd_vel_mux']['ros__parameters']

    # use_gazebo configures for Gazebo instead of the real robot
    use_gazebo = LaunchConfiguration('use_gazebo')
    # launch_gazebo actually launches Gazebo.  This can be false with use_gazebo
    # in case you want to run Gazebo on a more powerful dev machine while running
    # everything else on the robot computer.
    launch_gazebo = LaunchConfiguration('launch_gazebo')
    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_gazebo', 
            default_value='False',
            description='Set to True to use Gazebo instead of real robot.'
        ),

        DeclareLaunchArgument(
            name='launch_gazebo', 
            default_value='False',
            description='Set to True to launch Gazebo.'
        ),

        DeclareLaunchArgument(
            name='world_sdf_file', 
            default_value='my_world4.sdf',
            description='Default world SDF file when using Gazebo'
        ),

        DeclareLaunchArgument(
            name='gps_origin_lat', 
            default_value='30.609866',
            description='Latitude of origin when using GPS with Gazebo'
        ),

        DeclareLaunchArgument(
            name='gps_origin_lon', 
            default_value='-96.340424',
            description='Longitude of origin when using GPS with Gazebo'
        ),

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

        DeclareLaunchArgument(
            name='use_rosbridge', 
            default_value='True',
            description='Set to True launch ROS bridge'
        ),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_gazebo')),

        Node(
            condition=UnlessCondition(LaunchConfiguration('use_gazebo')),
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration('base_serial_port')]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('launch_gazebo')),
            launch_arguments={
                'use_gps': LaunchConfiguration('use_gps'),
                'world_sdf_file': LaunchConfiguration('world_sdf_file'),
                'gps_origin_lat': LaunchConfiguration('gps_origin_lat'),
                'gps_origin_lon': LaunchConfiguration('gps_origin_lon')
            }.items()
        ),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': 'true',
                'publish_joints': 'true'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch_path),
            condition=UnlessCondition(LaunchConfiguration('use_gazebo')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
            condition=IfCondition(LaunchConfiguration('joy')),
            # Remap the output according to the cmd_vel_mux config
            launch_arguments={
                'cmd_vel_topic': 'cmd_vel/joy'
            }.items()
        ),

        # Setup a mux to control the source of cmd_vel messages sent to the base
        Node(
            package='cmd_vel_mux',
            executable='cmd_vel_mux_node',
            output='screen',
            parameters=[cmd_vel_mux_config]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'navigation.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_nav')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_gazebo'),
                'map': LaunchConfiguration('map'),
                'slam': LaunchConfiguration('slam'),
                'use_gps': LaunchConfiguration('use_gps')
            }.items()
        ),

        # Conditionally start the map servers for supporting a map keep-out area
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'keep_out_area.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_keep_out')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_gazebo'),
                'autostart': 'True'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'sensor_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_gazebo'),
                'use_gps': LaunchConfiguration('use_gps'),
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('elsabot_4wd'), 'launch', 'gps.launch.py')),
            condition=IfCondition(
                PythonExpression([
                    'not ',
                    use_gazebo,
                    ' and ',
                    use_gps
                ])
            ),
            launch_arguments={
               'use_gps_rtk': LaunchConfiguration('use_gps_rtk')
            }.items(),
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': LaunchConfiguration('use_gazebo')}]
        ),

        # web bridge (for proxying topics/actions to/from ros_web based applications)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_path),
            condition=IfCondition(LaunchConfiguration('use_rosbridge'))
        )
    ])
