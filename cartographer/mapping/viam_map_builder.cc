/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/viam_map_builder.h"
#include "cartographer/common/config.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/viam_read_PCD_file.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/internal/testing/mock_map_builder.h"
#include "cartographer/mapping/internal/testing/mock_pose_graph.h"
#include "cartographer/mapping/internal/testing/mock_trajectory_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "gmock/gmock.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "gtest/gtest.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};
double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.


void MapBuilderViam::SetUp(std::string configuration_directory, std::string configuration_basename) {

  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string lua_code =
      file_resolver->GetFileContentOrDie(configuration_basename);

  auto options = cartographer::common::LuaParameterDictionary::NonReferenceCounted(lua_code, std::move(file_resolver));

  auto map_builder_parameters = options->GetDictionary("map_builder");
  auto trajectory_builder_parameters = options->GetDictionary("trajectory_builder");

  map_builder_options_ = cartographer::mapping::CreateMapBuilderOptions(map_builder_parameters.get());
  trajectory_builder_options_ = cartographer::mapping::CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());

  return;
  }

void MapBuilderViam::BuildMapBuilder() {
    map_builder_ = CreateMapBuilder(map_builder_options_);
  }

void MapBuilderViam::SetOptionsTo3D() {
    map_builder_options_.set_use_trajectory_builder_2d(false);
    map_builder_options_.set_use_trajectory_builder_3d(true);
  }

void MapBuilderViam::SetOptionsToTSDF2D() {
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_submaps_options()
        ->mutable_range_data_inserter_options()
        ->set_range_data_inserter_type(
            proto::RangeDataInserterOptions::TSDF_INSERTER_2D);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_submaps_options()
        ->mutable_grid_options_2d()
        ->set_grid_type(proto::GridOptions2D::TSDF);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_ceres_scan_matcher_options()
        ->set_occupied_space_weight(10.0);
    map_builder_options_.mutable_pose_graph_options()
        ->mutable_constraint_builder_options()
        ->mutable_ceres_scan_matcher_options()
        ->set_occupied_space_weight(50.0);
  }

void MapBuilderViam::SetOptionsEnableGlobalOptimization() {
    map_builder_options_.mutable_pose_graph_options()
        ->set_optimize_every_n_nodes(3);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_motion_filter_options()
        ->set_max_distance_meters(0);
  }

MapBuilderInterface::LocalSlamResultCallback MapBuilderViam::GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
      local_slam_result_poses_.push_back(local_pose);
    };
  }

cartographer::sensor::TimedPointCloudData MapBuilderViam::GenerateSavedRangeMeasurements(std::string data_directory, std::string initial_filename, int i) {
    return GenerateSaved2DRangeMeasurements(initial_filename, i, data_directory);
  }

cartographer::sensor::TimedPointCloudData MapBuilderViam::GenerateSaved2DRangeMeasurements(std::string initial_filename, int i, std::string data_directory) {
    cartographer::sensor::TimedPointCloudData point_cloud_data = MapBuilderViam::GetDataFromFile(data_directory, initial_filename, i);

    LOG(INFO) << "----------PCD-------";
    LOG(INFO) << "Time: " << point_cloud_data.time;  
    LOG(INFO) << "Range (size): " << point_cloud_data.ranges.size();
    LOG(INFO) << "Range start (time): " << point_cloud_data.ranges[0].time;
    LOG(INFO) << "Range end (time): " << (point_cloud_data.ranges.back()).time;
    LOG(INFO) << "-----------------\n";

    return point_cloud_data;
  }

cartographer::sensor::TimedPointCloudData MapBuilderViam::GetDataFromFile(std::string data_directory, std::string initial_filename, int i) {
    cartographer::io::ReadFile read_file;
    std::vector<std::string> files;
    cartographer::sensor::TimedPointCloudData point_cloud;

    files = read_file.listFilesInDirectory(data_directory);

    if ( files.size() == 0 ) {
      LOG(INFO) << "No files found in data directory\n"; 
      return point_cloud;
    }

    point_cloud = read_file.timedPointCloudDataFromPCDBuilder(files[i], initial_filename);
    return point_cloud;
  }

}  // namespace mapping
}  // namespace cartographer

