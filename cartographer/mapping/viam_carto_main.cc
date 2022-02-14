#include "cartographer/mapping/viam_map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include <string>

#include "cartographer/io/submap_painter.h"
#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};


void PrintState(MapBuilderViam* mapBuilderViam, int trajectory_id, std::vector<transform::Rigid3d> final_poses_non_optimized) {
  const auto trajectory_nodes = mapBuilderViam->map_builder_->pose_graph()->GetTrajectoryNodes();
  const auto submap_data = mapBuilderViam->map_builder_->pose_graph()->GetAllSubmapData();

  const transform::Rigid3d final_pose = mapBuilderViam->map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) * mapBuilderViam->local_slam_result_poses_.back();

  LOG(INFO) << "----------------- POSES -----------------------";
  for (int i = 0; i < int(final_poses_non_optimized.size()); i++ ) { // file_list.size()
    LOG(INFO) << "Pose: "  << final_poses_non_optimized[i];
    //std::cout << "Pose: " << final_poses_non_optimized[i] << "\t | \t" << final_poses_optimized[i] << "std::endl";

  }
  LOG(INFO) << "-----------------------------------------------";
  LOG(INFO) << "Final Pose: " << final_pose << std::endl;
  LOG(INFO) << "-----------------------------------------------";

  return;
}

void PaintMap(std::unique_ptr<cartographer::mapping::MapBuilderInterface> & map_builder_, int i) {
  const double kPixelSize = 0.01;
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;

  LOG(INFO) << "size of submap_poses: " << submap_poses.size();
  if (submap_poses.size() > 0) {
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
      //std::cout << "width, height: " << submap_slice.width << ", " << submap_slice.height << std::endl;
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
}

void Run(std::string mode, std::string data_directory, const std::string& configuration_directory,
         const std::string& configuration_basename) {

  MapBuilderViam mapBuilderViam;

  // Add configs
  mapBuilderViam.SetUp(configuration_directory, configuration_basename);

  if (mode == "Global2D") {
    mapBuilderViam.SetOptionsEnableGlobalOptimization();
  }

  // Build MapBuilder
  mapBuilderViam.BuildMapBuilder();

  // Build TrajectoryBuilder
  int trajectory_id = mapBuilderViam.map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, mapBuilderViam.trajectory_builder_options_,
      mapBuilderViam.GetLocalSlamResultCallback());

  LOG(INFO) << "Trajectory ID: " << trajectory_id;

  TrajectoryBuilderInterface* trajectory_builder = mapBuilderViam.map_builder_->GetTrajectoryBuilder(trajectory_id);

  cartographer::io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  //std::vector<transform::Rigid3d> final_poses_non_optimized;
  std::vector<transform::Rigid3d> final_poses_non_optimized;

  LOG(INFO) << "Beginning to add data...";
  
  int j = 1;
  for (int i = 0; i < int(file_list.size()); i++ ) { // file_list.size()
    auto measurement = mapBuilderViam.GenerateSavedRangeMeasurements(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
        PaintMap(mapBuilderViam.map_builder_, j++);
    }
  }

  mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilderViam.map_builder_, 0);

  if (mode == "Global2D") {
      const auto trajectory_nodes = mapBuilderViam.map_builder_->pose_graph()->GetTrajectoryNodes();
      const auto submap_data = mapBuilderViam.map_builder_->pose_graph()->GetAllSubmapData();

      const transform::Rigid3d final_pose =
          mapBuilderViam.map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) * mapBuilderViam.local_slam_result_poses_.back();
  }

  PrintState(&mapBuilderViam, trajectory_id, final_poses_non_optimized);
  
  return;
}

}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {

  std::string FLAGS_configuration_directory = "../configuration_files";
  std::string FLAGS_configuration_basename = "viam_rplidar.lua";

  std::string mode = "Global2D";

  google::InitGoogleLogging("XXX");
  google::SetCommandLineOption("GLOG_minloglevel", "2");

  std::string data_directory;
  if (argc >= 2) {
    data_directory = argv[1];
  }
  else {
    data_directory = "/home/jeremyhyde-viam/data";
    LOG(INFO) << "No data directory specified, using default: " << data_directory << std::endl;
  }

  cartographer::mapping::Run(mode, data_directory, FLAGS_configuration_directory, FLAGS_configuration_basename);

  return 1;
}
