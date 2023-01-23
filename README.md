# rover_ros2_ws



To install:
- install ros2 (desktop)
- ```sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gazebo-ros2-control```
- git clone this repository
- ```cd rover_ros2_ws```
- ```git submodule init```
- ```git submodule update```
- ```colcon build --symlink-install```

To launch simulation:
- ```ros2 launch rover_bringup sim.launch.py```

To launch rover hardware:
- ```ros2 launch rover_bringup rover.launch.py```

finally, launch teleop of choice and remap ```cmd_vel:=rover_velocity_controller/cmd_vel_unstamped```