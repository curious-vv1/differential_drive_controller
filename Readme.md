This is a ROS humble and Gazebo Classic package which contains:
- ROS 2 C++ package for differential drive control
- Python script for waypoint navigation with PID
- Obstacle avoidance using lidar
- Working demonstration in Gazebo

# Setup Instructions

Make a workspace and src folder. In the src folder clone this repo using command   
`git clone https://github.com/curious-vv1/differential_drive_controller.git`

Use the command  
`colcon build`  
to make build, install and log files in the ros workspace.

Source the files using the command  
`source install/setup.bash`

# Steps to build and run the simulation

Run the following command to launch the differential drive bot in gazebo  
`ros2 launch differential_drive_controller display.launch.py`

Run the following command to do waypoint navigation to 2 place with obstacle avoidance:    
`ros2 run differential_drive_controller waypoint_navigation.py --ros-args -p waypoint_1_x:=0.0 -p waypoint_1_y:=-2.0 -p waypoint_2_x:=0.0 -p waypoint_2_y:=-10.0`

The points can be changed to whatever point desired to be navigated.

Run the following command to publish the right wheel rpm and left wheel rpm:   
`ros2 run differential_drive_controller rpm`

The following commands can be used to observe the rpms of wheels:    
`ros2 topic echo /left_wheel_rpm`  
`ros2 topic echo /right_wheel_rpm`

Run the following command to reset gazebo:    
`ros2 service call /reset_simulation std_srvs/srv/Empty`

