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

#include "cairo/cairo.h"
#include "glog/logging.h"
#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"

#include "cartographer/metrics/register.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/transform/transform.h"
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

  // --- NOTE: DO WE NEED THIS? ----
  // cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  // map_builder_options.set_collate_by_trajectory(true);


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
                       InsertionResult> insertion_result) {
      // DO WE NEED THIS?
      // OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      local_slam_result_poses_.push_back(local_pose);
      if (insertion_result != nullptr) {
        std::cout << "callback! " << insertion_result->insertion_submaps.size() << std::endl;
      }
    };
  }

void PaintMap(std::unique_ptr<cartographer::mapping::MapBuilderInterface> & map_builder_, int i) {
  const double kPixelSize = 0.05;
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;

  std::cout << "size of submap_poses: " << submap_poses.size() << std::endl;
  for (const auto& submap_id_pose : submap_poses) {
    cartographer::mapping::proto::SubmapQuery::Response response_proto;
    const std::string error = map_builder_->SubmapToProto(submap_id_pose.id, &response_proto);

    auto submap_textures = absl::make_unique<::cartographer::io::SubmapTextures>();
    submap_textures->version = response_proto.submap_version();
    for (const auto& texture_proto : response_proto.textures()) {
      const std::string compressed_cells(texture_proto.cells().begin(),
                                         texture_proto.cells().end());
      submap_textures->textures.emplace_back(::cartographer::io::SubmapTexture{
          ::cartographer::io::UnpackTextureData(compressed_cells, texture_proto.width(),
                                                texture_proto.height()),
          texture_proto.width(), texture_proto.height(), texture_proto.resolution(),
          cartographer::transform::ToRigid3(texture_proto.slice_pose())});
    }

    // Prepares SubmapSlice
    ::cartographer::io::SubmapSlice& submap_slice = submap_slices[submap_id_pose.id];
    const auto fetched_texture = submap_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    std::cout << "width, height: " << submap_slice.width << ", " << submap_slice.height << std::endl;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha, fetched_texture->width,
        fetched_texture->height, &submap_slice.cairo_data);
  }
  // Generates occupancy grid as CreateOccupancyGridMsg()
  cartographer::io::PaintSubmapSlicesResult painted_slices =
      PaintSubmapSlices(submap_slices, kPixelSize);
  auto image = cartographer::io::Image(std::move(painted_slices.surface));
  auto file = cartographer::io::StreamFileWriter("pictures/map_" + std::to_string(i) + ".png");
  image.WritePng(&file);
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

  int i = 1;
  for (const auto& measurement : measurements) {
    // TODO: Grab our pointcloud 
    // e.g.: measurement = Rplidar.NextPointCloud()
    // TODO: Transform our pointcloud to theirs 
    trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    for (const auto& submap_id_pose : map_builder_->pose_graph()->GetAllSubmapPoses()) {
      // std::cout << submap_id_pose << ", " << std::endl;
      std::cout << "submapidpose! " << std::endl;
    }
    PaintMap(map_builder_, i++);
  }
  map_builder_->FinishTrajectory(trajectory_id);
  map_builder_->pose_graph()->RunFinalOptimization();

  PaintMap(map_builder_, 0);
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