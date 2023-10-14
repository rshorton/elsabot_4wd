from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

def generate_launch_description():
    
    use_gps = LaunchConfiguration('use_gps')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_gps', 
            default_value='False',
            description='Set to True to use GPS'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        SetParameter(name='use_sim_time', value=LaunchConfiguration("use_sim_time")),

        # IMU data filtering
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('elsabot_4wd'), 'config', 'imu_filter.yaml'])
            ],
            remappings=[
                # Sub
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/mag', 'imu/mag'),
                # Pub
                ('imu/data', 'imu/data')
            ]
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

        #
        # Robot localization doc:  http://docs.ros.org/en/noetic/api/robot_localization/html/index.html
        #
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
                # navsat_transform_node uses imu contrary to doc
                ('imu', 'imu/data'),
                ('odometry/filtered', 'odometry/global'),
                # Pubs
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
