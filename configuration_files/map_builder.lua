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

include "pose_graph.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = true,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false,
}


-- NOTE: Choose the next two values based on your dataset.
-- These have to be most like tuned for each dataset until we find a better solution.
-- ------------------------------------------------------------------------
-- Example values:
-- For fewer data points (e.g. ~1300 scans for the "office 3 area"), between 0.3 and 1. are good values.
-- 0.3 is much slower, but results in slightly improved accuracy.
-- 1. is a good tradeoff between time and accuracy.
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 1
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 1

-- For denser data points (e.g. ~2830 scans for the "office 3 area")
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 1e8
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 1e8
-- ------------------------------------------------------------------------


-- NOTE: The following lines helped make the example with ~2830 scans for the "office 3 area",
-- but were commented out for the dataset with ~1300 scans.
-- ------------------------------------------------------------------------
MAP_BUILDER.pose_graph.constraint_builder.min_score=0.3
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.7
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 10
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 2
MAP_BUILDER.pose_graph.global_constraint_search_after_n_seconds = 3
MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 100
-- ------------------------------------------------------------------------

return MAP_BUILDER
