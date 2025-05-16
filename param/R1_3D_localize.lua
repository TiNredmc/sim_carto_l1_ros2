include "R1_3D_mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 10,
}
-- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100 -- 300

-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 1.0

-- POSE_GRAPH.optimize_every_n_nodes = 2
POSE_GRAPH.matcher_translation_weight = 1e2
-- POSE_GRAPH.matcher_rotation_weight = 1.6e7

-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- 0.0001
-- POSE_GRAPH.global_sampling_ratio = 0.000009 -- 0.001

-- POSE_GRAPH.constraint_builder.min_score = 0.50 -- 0.48
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55 --0.55
-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e3 -- last good 1.1e3
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.1e3 -- last good 1.1e3

-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0 -- 15
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(45.)
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 4

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e-10 -- 3.1e4
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2

-- POSE_GRAPH.global_constraint_search_after_n_seconds = 5
-- POSE_GRAPH.max_num_final_iterations = 50

return options
