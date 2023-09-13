-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

robot_type = os.getenv("robot_name")

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = robot_type.."/base_footprint", --imu_link
  published_frame = robot_type.."/odom", --base_footprint
  odom_frame = robot_type.."/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true, --false,
  use_nav_sat = false,
  use_landmarks = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1, --10
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.3,
  submap_publish_period_sec = 0.3, --0.3
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0, --0.5
  odometry_sampling_ratio = 0.2, --0.2
  fixed_frame_pose_sampling_ratio = 0.5, --1.0
  imu_sampling_ratio = 0.1,
  landmarks_sampling_ratio = 0.5, --1.0
  publish_tracked_pose = true, --add Enable publishing of tracked pose as a geometry_msgs/PoseStamped to topic ?tracked_pose
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER.collate_landmarks = false -- set this to false if publishing sparse landmarks
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
-- slightly slower insertion
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.53
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.493

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false --false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2

--POSE_GRAPH.optimize_every_n_nodes = 20 --add
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options

