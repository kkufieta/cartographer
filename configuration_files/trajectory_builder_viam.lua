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

include "trajectory_builder.lua"
TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
TRAJECTORY_BUILDER.collate_fixed_frame = false
TRAJECTORY_BUILDER.collate_landmarks = false

-- NOTE: true for the ~2830 scan data set, false for the ~1365 scan data set
TRAJECTORY_BUILDER.trajectory_builder_2d.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 1e5

-- Good range: between 0.3 and 1.0 (for data points up to ~1365)
TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.rotation_weight = 10.
TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.translation_weight = 10.

TRAJECTORY_BUILDER.trajectory_builder_2d.min_range = 0.2
TRAJECTORY_BUILDER.trajectory_builder_2d.max_range = 25.
TRAJECTORY_BUILDER.trajectory_builder_2d.missing_data_ray_length = 25.5

-- NOTE: Not sure if this is helpful to set or not, and which value is good to use. 
TRAJECTORY_BUILDER.trajectory_builder_2d.num_accumulated_range_data = 1


return TRAJECTORY_BUILDER
