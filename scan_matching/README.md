First run the simulation using:

```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

then run the the scan_matching for localisation using:

```
ros2 run scan_matching scanmatch_node
```

Then for the visualization

```
ros2 run rviz2 rviz2
```

In Rviz2, add these displays:
- Add a "TF" display to see the coordinate frames
- Add a "Maker" display for the point clouds
  1) Set its topic to "/scan_match_debug"
- Add a "Pose" display
  1) Set its topic to "/scan_match_location"
-Set the Fixed Frame to "map"or "ego_racecar/laser"
