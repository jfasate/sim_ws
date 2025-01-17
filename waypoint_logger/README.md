First run the simulation using:
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Now launch the keyboard:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

We have to get the waypoint using:
```
ros2 run waypoint_logger waypoint_logger_node.py
```
