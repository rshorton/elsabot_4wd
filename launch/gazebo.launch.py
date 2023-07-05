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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("elsabot_4wd"), "urdf/robots", f"4wd.urdf.xacro"]
    )

    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'sensors.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('elsabot_4wd'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("elsabot_4wd"), "config", "ekf.yaml"]
    )

    params = {'use_sim_time': True, 'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])}

    # Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            #'gz_args': '-r empty.sdf'
            'gz_args': '-r ' +  os.path.join(get_package_share_directory('elsabot_4wd'), 'gazebo_worlds', 'my_world4.sdf')
        }.items(),
    )

    # RViz
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'robot_description_publisher.rviz')
        ]
    )

    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'elsabot_4wd',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '-0.02',
                    '-topic', '/robot_description'
                    ],
                 output='screen')

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                   '/model/elsabot_4wd/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/world/empty/model/elsabot_4wd/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

                   ],
        parameters=[{'qos_overrides./model/elsabot_4wd.subscriber.reliability': 'reliable'}],
        remappings=[('/world/empty/model/elsabot_4wd/joint_state', 'joint_states'),
                    ('/model/elsabot_4wd/tf', 'tf'),
                    #('/odom', 'odom/unfiltered')
                   ],
        output='screen'
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'publish_joints': 'true',
        }.items()
    )

    cmd_timeout = Node(
        package='cmd_vel_timeout',
        executable='command_timeout',
        name='command_timeout_node',
        output='screen',
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path
        ],
        remappings=[("odometry/filtered", "odom")]
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),

        gazebo,
        spawn,
        bridge,
        description,
        cmd_timeout,
        #robot_localization,
        #rviz
    ])
