# Elsabot 4WD Robot

Top-level bring up scripts for the 4WD base version of the Elsabot robot.  

Some parts of these scripts and configuration files are from the Linorobot2 project:  https://github.com/linorobot/linorobot2

The firmware for the base microcontroller is based on the linorobot2_hardware project.  See this fork for the firmware used for the Elsabot 4WD base (master branch): https://github.com/rshorton/linorobot2_hardware.

### Installation

ROS Humble
* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
    
* The desktop install was used, but a minimal install should work also (and then run RVIZ on your devel computer instead).

Nav2
* sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup 

micro-ROS and agent
* Used the 'Building' steps from https://github.com/micro-ROS/micro_ros_setup/blob/humble/README.md

Other
* sudo apt install ros-humble-rplidar-ros
* sudo apt install ros-humble-laser-filters
* sudo apt install ros-humble-joint-state-publisher
* sudo apt install ros-humble-rosbridge-suite
* sudo apt install ros-humble-robot-localization
* sudo apt install ros-humble-joy-linux
* (probably others, sorry)

Development Machine
* Installed full ROS Humble install
* Used RVIZ on this machine.

## Other Setup on Robot computer

udev rules, /etc/udev/rules.d/99-usb-serial.rules 

* SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0",ATTRS{idProduct}=="0483", SYMLINK+="teensy"
* SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"

teleop Controller
* Luna game controller in bluetooth mode paired to RPi4.

## Running

1. Start base functionality using shell 1:
    + ros2 launch elsabot_4wd bringup.launch.py
    + Use teleop with game controller to control jeep
2. Start navigation using shell 2:
    + ros2 launch elsabot_4wd navigation.launch.py
3. Start RVIZ on development host  
    + rviz2
    + Use RVIZ to set initial position and then issue nav commands.
  
Map creation for navigation
  1. Use this step instead of 2 above:
     + ros2 launch elsabot_4wd slam.launch.py rviz:=false
  2. Move robot around using telop to build map.
  3. Save map using:
     + ros2 run nav2_map_server map_saver_cli -f <path_to/elsabot_4wd/maps/your_map_name> --ros-args -p save_map_timeout:=10000.
  4. Edit launch/navigation.launch.py to use your map.

 
## Gazebo Simulation (Using Fortress)

Steps for running with Gazebo. 

1. Start base functionality using shell 1:
    + export IGN_GAZEBO_RESOURCE_PATH=<path_to workspace/src>
    + ros2 launch elsabot_4wd gazebo.launch.py
2. Start navigation using shell 2:
    + ros2 launch elsabot_4wd navigation.launch.py use_sim_time:=true
3. Start RVIZ on development host  
    + rviz2
    + <Use RViz to set the 2D pose estimate.>

To run Gazebo with simulated GPS instead of AMCL, revise the launch commands as:

* ros2 launch elsabot_4wd gazebo.launch.py use_gps:=True
* ros2 launch elsabot_4wd navigation.launch.py use_sim_time:=true use_gps:=True

(No need to set the initial position using Rviz in this case.)