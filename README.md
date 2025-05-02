# sim_carto_l1_ros2

sim_carto_l1_ros2 is a ros2 package to launch the offcial Unitree's ros bag recording of the Unitree 4D L1 lidar and do a semi-online Cartographer ros SLAM mapping session.

Launching
=

To launch the mapping
```
ros2 launch sim_carto_l1_ros2 simbag.launch.py
```

To generates the pbstream map which will be save at your home directory
```
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: l2.pbstream}"
```

To generates the PCD point cloud file, first copy your own pbstream map to replace one inside this package, colcon build then run
```
ros2 launch sim_carto_l1_ros2 assets_writer3d.launch.py
```
Point cloud output should be inside your home folder

Tuning
=

Further tuning of the Cartographer can be made. First start from the correct URDF transform. Since Unitree didn't provide the coordinate of the imu and laser scan relative to the lidar base. I have to guess it from the Unitree fork of Point-LIO package.  
  
Currently the Cartographer was tuned to have number of range data (submap.num_range_data) of 150. This can be reduced later in the localization mode or other scenarios. The global slam was tuned while running playback speed of x2. The result in the video is the REALTIME (running x1).
[![Cartographer ros with Unitree 4D L1](https://img.youtube.com/vi/FdZ6XQt9f6c/0.jpg)](https://www.youtube.com/watch?v=FdZ6XQt9f6c)
