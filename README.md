## PqaBOT ROS2 Humble
ROS2 Training repository

#### Launch robot_state_publisher, Rviz2, & Gazebo
```
ros2 launch pqa_bot launch_sim.launch.py
```
#### Drive robot with joystick
```
ros2 run joy joy_node dev:=/dev/input/js0 
ros2 run teleop_twist_joy teleop_node
```

#### Launch the robot with a gazebo world
```
ros2 launch pqa_bot launch_sim.launch.py world:=./src/pqa_bot/worlds/obstacles.world
```

#### Check if robot receive move input
```
ros2 topic echo /cmd_vel
```