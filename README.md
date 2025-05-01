# sim_carto_l1_ros2

sim_carto_l1_ros2 is a ros2 package to launch the offcial Unitree's ros bag recording of the Unitree 4D L1 lidar and do a semi-online Cartographer ros SLAM mapping session.

Launching
=

To launch the mapping, runs
```
ros2 launch sim_carto_l1_ros2 simbag.launch.py
```

Tuning
=

Further tuning of the Cartographer can be made. First start from the correct URDF transform. Since Unitree didn't provide the coordinate of the imu and laser scan relative to the lidar base. I have to guess it from the Unitree fork of Point-LIO package.  
  
Currently the Cartographer was tuned to have number of range data (submap.num_range_data) of 150. This can be reduced later in the localization mode or other scenarios. The global slam was tuned while running playback speed of x2. The result in the video is the REALTIME (running x1).
