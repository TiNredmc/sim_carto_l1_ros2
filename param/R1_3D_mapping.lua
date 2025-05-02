include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map",  -- map frame name 
  tracking_frame = "unilidar_imu",  -- tracking frame name
  published_frame = "base_link",  -- published frame name 
  odom_frame = "odom",  -- name of the odometer frame
  provide_odom_frame = true,  -- whether to provide the odometer frame
  publish_frame_projected_to_2d = false,  -- whether to publish 2d gesture  
  use_pose_extrapolator = false,
  use_odometry = false,  -- whether use odometry
  use_nav_sat = false,  -- whether use the navigation satellite 
  use_landmarks = false,  -- whether use the landmark
  num_laser_scans = 0,  -- LiDAR number  
  num_multi_echo_laser_scans = 0,  -- number of multi-echo LiDAR  
  num_subdivisions_per_laser_scan = 1,  -- number of subdivisions for each laser scan
  num_point_clouds = 1,  -- Single point cloud source from the Unitree 4D L1 PM lidar
  lookup_transform_timeout_sec = 0.5,  -- timeout for finding transformations (seconds)  
  submap_publish_period_sec = 0.3,  -- submap release cycle (seconds)
  pose_publish_period_sec = 5e-3,  -- attitude release period (seconds)
  publish_tracked_pose = true, -- publish tracked pose (trajectory server)
  trajectory_publish_period_sec = 40e-3,  -- trajectory release period (seconds)
  rangefinder_sampling_ratio = 1.0,  -- rangefinder sampling ratio
  odometry_sampling_ratio = 1.0,  -- odometer sampling rate
  fixed_frame_pose_sampling_ratio = 1.,  -- fixed frame attitude sampling ratio  
  imu_sampling_ratio = 1.,  -- IMU sampling ratio
  landmarks_sampling_ratio = 1.,  -- landmarks sampling ratio
}
 
MAP_BUILDER.num_background_threads = 6
 
MAP_BUILDER.use_trajectory_builder_3d = true  -- use 3D tranjectory builder
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1 -- No point cloud accumulation since one pointcloud2 message contains one revolution scan of the Unitree L1
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 120 -- 150
TRAJECTORY_BUILDER_3D.min_range = 0.1  -- ignore anything smaller than the robot radius, limiting it to the minimum scan range of the lidar
TRAJECTORY_BUILDER_3D.max_range = 30.0  -- the maximum scanning range of the lidar

TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false  -- Whether to scan for matches using real-time loopback detection
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 5.0
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(180.)
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e4 -- 1e-1

TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10
-- TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 5e0 -- seems to affect the elevation changes (more weight, more z worping)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 9.5e0 -- 5e0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 2e2 -- last best a2
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100

-- TODO - Tune the backend slam, running only the front end somehow produce the better result. The backend slam kind of screw up.

POSE_GRAPH.optimize_every_n_nodes = 40 -- 20

-- POSE_GRAPH.global_sampling_ratio = 0.05

POSE_GRAPH.log_residual_histograms  = false

-- Non-loop closure constraints
-- POSE_GRAPH.matcher_translation_weight = 2.5e2 -- default 5e2 
-- POSE_GRAPH.matcher_rotation_weight = 0.825e8 -- default 0.825e8
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 0.5e3 -- last good 1.1e3
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e-2

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 1.0
POSE_GRAPH.constraint_builder.min_score = 0.52  -- 0.52
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 16
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.50
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.35
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.5
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.)

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 4
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 4e0
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 6e0
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 1
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 2e0

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
-- POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
-- POSE_GRAPH.optimization_problem.rotation_weight = 100
POSE_GRAPH.global_constraint_search_after_n_seconds = 3
-- POSE_GRAPH.max_num_final_iterations = 50

return options