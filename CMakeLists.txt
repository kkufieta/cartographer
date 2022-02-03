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

#include <condition_variable>
#include <mutex>

#include "cartographer/cloud/client/map_builder_stub.h"
#include "cartographer/cloud/internal/map_builder_server.h"
#include "cartographer/io/internal/in_memory_proto_stream.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/internal/testing/mock_map_builder.h"
#include "cartographer/mapping/internal/testing/mock_pose_graph.h"
#include "cartographer/mapping/internal/testing/mock_trajectory_builder.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/mapping/map_builder.h"
#include "glog/logging.h"

using ::cartographer::mapping::TrajectoryBuilderInterface;
using SensorId = ::cartographer::mapping::TrajectoryBuilderInterface::SensorId;

namespace cartographer {
namespace cloud {
namespace {

constexpr char kClientId[] = "CLIENT_ID";
constexpr char server_address[] = "0.0.0.0:50051";
const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
constexpr double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.

constexpr char kSerializationHeaderProtoString[] = "format_version: 2";
constexpr char kPoseGraphProtoString[] = R"(pose_graph {
      trajectory: {
        trajectory_id: 0
		node: {}
		submap: {}
	  }
    })";
constexpr char kAllTrajectoryBuilderOptionsProtoString[] =
    R"(all_trajectory_builder_options {
      options_with_sensor_ids: {}
    })";
constexpr char kSubmapProtoString[] = "submap {}";
constexpr char kNodeProtoString[] = "node {}";
constexpr char kTrajectoryDataProtoString[] = "trajectory_data {}";
constexpr char kOdometryDataProtoString[] = "odometry_data {}";
constexpr char kFixedFramePoseDataProtoString[] = "fixed_frame_pose_data {}";
constexpr char kLandmarkDataProtoString[] = "landmark_data {}";

class KatClient {
 public:
  void InitializeStub() {
    stub_ = absl::make_unique<MapBuilderStub>(
        server_address, kClientId);
  }

  void SetOptionsToTSDF2D() {
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_submaps_options()
        ->mutable_range_data_inserter_options()
        ->set_range_data_inserter_type(
            ::cartographer::mapping::proto::RangeDataInserterOptions::
                TSDF_INSERTER_2D);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_submaps_options()
        ->mutable_grid_options_2d()
        ->set_grid_type(::cartographer::mapping::proto::GridOptions2D::TSDF);
    trajectory_builder_options_.mutable_trajectory_builder_2d_options()
        ->mutable_ceres_scan_matcher_options()
        ->set_occupied_space_weight(10.0);
    // map_builder_server_options_.mutable_map_builder_options()
    //     ->mutable_pose_graph_options()
    //     ->mutable_constraint_builder_options()
    //     ->mutable_ceres_scan_matcher_options()
    //     ->set_occupied_space_weight(50.0);
  }

  void WaitForLocalSlamResults(size_t size) {
    std::unique_lock<std::mutex> lock(local_slam_result_mutex_);
    local_slam_result_condition_.wait(
        lock, [&] { return local_slam_result_poses_.size() >= size; });
  }

  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options_;
  std::unique_ptr<MapBuilderStub> stub_;
  TrajectoryBuilderInterface::LocalSlamResultCallback
      local_slam_result_callback_;
  std::condition_variable local_slam_result_condition_;
  std::mutex local_slam_result_mutex_;
  std::vector<transform::Rigid3d> local_slam_result_poses_;
  int number_of_insertion_results_;
};

}  // namespace
}  // namespace cloud
}  // namespace cartographer


int main(int argc, char** argv) {
  const std::string kTrajectoryBuilderLua = R"text(
    include "trajectory_builder.lua"
    TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
    TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 4
    return TRAJECTORY_BUILDER)text";
  auto trajectory_builder_parameters = ::cartographer::mapping::testing::ResolveLuaParameters(kTrajectoryBuilderLua);
  auto trajectory_builder_options_ = ::cartographer::mapping::CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());
  
  auto number_of_insertion_results_ = 0;

  auto local_slam_result_callback_ =
      [this](int, common::Time, transform::Rigid3d local_pose,
              sensor::RangeData,
              std::unique_ptr<
                  const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult>
                  insertion_result) {
        std::unique_lock<std::mutex> lock(local_slam_result_mutex_);
        if (insertion_result) {
          ++number_of_insertion_results_;
        }
        local_slam_result_poses_.push_back(local_pose);
        lock.unlock();
        local_slam_result_condition_.notify_all();
      };



  InitializeStub();
  int trajectory_id =
      stub_->AddTrajectoryBuilder({kRangeSensorId}, trajectory_builder_options_,
                                  local_slam_result_callback_);
  TrajectoryBuilderInterface* trajectory_stub =
      stub_->GetTrajectoryBuilder(trajectory_id);
  const auto measurements = ::cartographer::mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);

  // OR:
  // sensor::ImuData imu_data{common::FromUniversal(42),
  //                          Eigen::Vector3d(0., 0., 9.8),
  //                          Eigen::Vector3d::Zero()};
  // trajectory_stub->AddSensorData(kImuSensorId.id, imu_data);
  for (const auto& measurement : measurements) {
    trajectory_stub->AddSensorData(kRangeSensorId.id, measurement);
  }
  WaitForLocalSlamResults(measurements.size());

  stub_->FinishTrajectory(trajectory_id);
  stub_->pose_graph()->RunFinalOptimization();
  server_->Shutdown();
}





My only question regarding this is whether or not for the rplidar it is then better to do a deep dive into the c++ code to figure out to detach it from the rplidar repo and integrate directly or to still use it as