# Launch GPS related nodes:
# * Driver to read GPS NMEA sentences from the GPS module and publish GPS fix to /gps/fix
# * Optionally run NTRIP client to fetch and send RTCM data to GPS module for RTK suport

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gps_serial_port_nmea', 
            default_value='/dev/PX1122R_gps',
            description='Serial port connected to GPS module'
        ),

        DeclareLaunchArgument(
            name='gps_serial_baud_rate_nmea', 
            default_value='115200',
            description='Baud rate of GPS serial port'
        ),

        DeclareLaunchArgument(
            name='gps_rtcm_input_desc', 
            default_value='serial://PX1122R_gps_rtcm:115200#rtcm1',
            description='str2str output description'
        ),

        DeclareLaunchArgument(
            name='ntrip_caster_url', 
            default_value='ntrip://elsabot:ebot1@192.168.86.145:2101/mcktx:rtcm1',
            description='NTRIP caster URL'
        ),

        DeclareLaunchArgument(
            name='use_rtk', 
            default_value='False',
            description='Set to True to use NTRIP client for RTK'
        ),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration("gps_serial_port_nmea")},
                {'baud': LaunchConfiguration("gps_serial_baud_rate_nmea")},
                {'frame_id': 'gps_link'}
            ],
            remappings=[('fix', 'gps/fix')]
        ),

        ExecuteProcess(
            cmd=['str2str', '-in', LaunchConfiguration('ntrip_caster_url'), '-out', LaunchConfiguration('gps_rtcm_input_desc')],
            condition=IfCondition(LaunchConfiguration("use_rtk"))
        )
    ])
