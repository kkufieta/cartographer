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

// Run this file like this:
// ./kat_main -configuration_directory=../configuration_files -configuration_basename=config_kat.lua

#include <vector>
#include <memory>
#include <iostream>

#include "glog/logging.h"
#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"

#include "cartographer/metrics/register.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "gflags/gflags.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
constexpr double kDuration = 4.;         // Seconds.
constexpr double kTimeStep = 0.1;        // Seconds.
constexpr double kTravelDistance = 1.2;  // Meters.

std::vector<::cartographer::transform::Rigid3d> local_slam_result_poses_;

std::unique_ptr<::cartographer::common::LuaParameterDictionary> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string lua_code =
      file_resolver->GetFileContentOrDie(configuration_basename);


  // Use this instead of NonReferenceCounted if you want to check lua files:
  // return absl::make_unique<::cartographer::common::LuaParameterDictionary>(
  //     lua_code, std::move(file_resolver));

  auto luaParameterDictionary = cartographer::common::LuaParameterDictionary::NonReferenceCounted(
      lua_code, std::move(file_resolver));
  return luaParameterDictionary;
}

cartographer::mapping::MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback() {
    return [=](const int trajectory_id, const ::cartographer::common::Time time,
               const ::cartographer::transform::Rigid3d local_pose,
               ::cartographer::sensor::RangeData range_data_in_local,
               const std::unique_ptr<
                   const cartographer::mapping::TrajectoryBuilderInterface::
                       InsertionResult>) {
      local_slam_result_poses_.push_back(local_pose);
    };
  }

void Run(const std::string& configuration_directory,
         const std::string& configuration_basename) {
  std::cout << "Start Run\n";
  auto options = LoadOptions(configuration_directory, configuration_basename);

  auto map_builder_parameters = options->GetDictionary("map_builder");
  auto trajectory_builder_parameters = options->GetDictionary("trajectory_builder");

  auto map_builder_options_ =
        cartographer::mapping::CreateMapBuilderOptions(map_builder_parameters.get());
  auto trajectory_builder_options_ =
        cartographer::mapping::CreateTrajectoryBuilderOptions(trajectory_builder_parameters.get());

  auto map_builder_ = cartographer::mapping::CreateMapBuilder(map_builder_options_);

  int trajectory_id = map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, trajectory_builder_options_,
      GetLocalSlamResultCallback());

  cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder =
      map_builder_->GetTrajectoryBuilder(trajectory_id);

  const auto measurements = cartographer::mapping::testing::GenerateFakeRangeMeasurements(
      kTravelDistance, kDuration, kTimeStep);

  for (const auto& measurement : measurements) {
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();
  std::cout << "End Run\n";
}

int main(int argc, char** argv) {

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  if (FLAGS_configuration_directory.empty()) {
    std::cout << "-configuration_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_configuration_basename.empty()) {
    std::cout << "-configuration_basename is missing.\n";
    return EXIT_FAILURE;
  }

  Run(FLAGS_configuration_directory, FLAGS_configuration_basename);
}