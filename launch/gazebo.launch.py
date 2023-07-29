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

# Various modifications by Scott Horton for Elsabot robots

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'urdf/robots', f"4wd.urdf.xacro"]
    )

    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'sensors.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'description.launch.py']
    )

    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument(
            name='use_gps', 
            default_value='False',
            description='Set to True to use GPS'
        ),

        # Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                #'gz_args': '-r empty.sdf'
                'gz_args': '-r ' +  os.path.join(get_package_share_directory('elsabot_4wd'), 'gazebo_worlds', 'my_world4.sdf')
            }.items(),
        ),   

        # Robot            
        Node(package='ros_gz_sim', executable='create',
            arguments=[
                '-name', 'elsabot_4wd',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '-0.02',
                '-topic', '/robot_description'
            ],
            output='screen'
        ),

        # Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom/unfiltered@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/model/elsabot_4wd/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/world/empty/model/elsabot_4wd/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat'
            ],
            parameters=[
                {'qos_overrides./model/elsabot_4wd.subscriber.reliability': 'reliable'}
            ],
            remappings=[
                ('/world/empty/model/elsabot_4wd/joint_state', 'joint_states'),
                ('elsabot_4wd/base_footprint/navsat', 'gps_link'),
                ('/navsat','/gps/fix'),
            ],
            output='screen'
        ),

        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': 'true',
                'publish_joints': 'true',
            }.items()
        ),

        # Node for sending cmd_vel 'stop' if no new cmd_vel updates
        Node(
            package='cmd_vel_timeout',
            executable='command_timeout',
            name='command_timeout_node',
            output='screen',
        ),

        # Robot localization for fusing IMU with base odometry (when not using GPS)
        Node(
            condition=IfCondition(
                PythonExpression([
                    'not ',
                    use_gps,
                ])
            ),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'ekf.yaml'])
            ],
            remappings=[
                ('odometry/filtered', 'odometry/local')
            ]
        ),

        # Fuse IMU with base odometry (when using GPS)
        Node(
            condition=IfCondition(LaunchConfiguration('use_gps')),
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'ekf_with_gps.yaml'])
            ],
            remappings=[
                ('odometry/filtered', 'odometry/local')
            ]
        ),

        # Fuse GPS with base odometry and IMU (when using GPS)
        Node(
            condition=IfCondition(LaunchConfiguration('use_gps')),
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_filter_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'ekf_with_gps.yaml'])
            ],
            remappings=[
                ('odometry/filtered', 'odometry/global')
            ]
        ),

        # Run NavSat transform for converting GPS coords into UTM coords
        # and offseting to the initial position
        Node(
            condition=IfCondition(LaunchConfiguration('use_gps')),
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'ekf_with_gps.yaml'])
            ],
            remappings=[
                # Subs
                ('gps/fix', 'gps/fix'),
                ('imu', 'imu/data'),
                ('odometry/filtered', 'odometry/global'),
                # Pubs
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
  