#include "cartographer/mapping/viam_map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include <string>

#include "cartographer/io/submap_painter.h"
#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"


DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(data_directory, "",
              "Directory in which rplidar data is expected.");
DEFINE_string(output_directory, "",
              "Directory where map images are saved in.");

namespace cartographer {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};


void PrintState(MapBuilderViam* mapBuilderViam, int trajectory_id, std::vector<transform::Rigid3d> final_poses_non_optimized) {
  const auto trajectory_nodes = mapBuilderViam->map_builder_->pose_graph()->GetTrajectoryNodes();
  const auto submap_data = mapBuilderViam->map_builder_->pose_graph()->GetAllSubmapData();

  const transform::Rigid3d final_pose = mapBuilderViam->map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) * mapBuilderViam->local_slam_result_poses_.back();

  std::cout << "----(non-opt)---- FINAL POSES ------(opt)------\n";
  for (int i = 0; i < int(final_poses_non_optimized.size()); i++ ) { // file_list.size()
    std::cout << "Pose: "  << final_poses_non_optimized[i] << "std::endl";
    //std::cout << "Pose: " << final_poses_non_optimized[i] << "\t | \t" << final_poses_optimized[i] << "std::endl";

  }
  std::cout << "-----------------------------------------------\n";
  std::cout << "Final Pose: " << final_pose << std::endl;
  std::cout << "-----------------------------------------------\n";
  std::cout << "Size of localSLAMResultPose = " << mapBuilderViam->local_slam_result_poses_.size() << std::endl;

  return;
}

void PaintMap(std::unique_ptr<cartographer::mapping::MapBuilderInterface> & map_builder_, std::string output_directory, int i) {
  const double kPixelSize = 0.01;
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;

  std::cout << "size of submap_poses: " << submap_poses.size() << std::endl;

  //const mapping::proto::PoseGraph& pose_graph = map_builder_->pose_graph();


  
  //const mapping::proto::Trajectory& trajectory = pose_graph.trajectory(0);

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


    // // -----------------------------
    // const mapping::proto::Trajectory& trajectory = mapBuilderViam->map_builder_->pose_graph()->trajectories[0];
    // const FloatColor& color = ;
    // const PoseToPixelFunction& pose_to_pixel = ;
    // cairo_surface_t* surface = ;

    // DrawTrajectory(trajectory, color, pose_to_pixel, surface);

    // // -----------------------------
    // const std::vector<mapping::proto::Trajectory>& trajectory = mapBuilderViam->map_builder_->pose_graph()->trajectories;
    // const Eigen::Array2i& offset = Eigen::Array2i::Zero()
    // const mapping::ProbabilityGrid& probability_grid = ;
    // cairo_surface_t* = ;

    // DrawTrajectoriesIntoImage(probability_grid, offset, trajectories, surface);

    auto image = cartographer::io::Image(std::move(painted_slices.surface));
    auto file = cartographer::io::StreamFileWriter(output_directory + "/map_" + std::to_string(i) + ".png");
    image.WritePng(&file);
  }
}

void Run(std::string mode,
        std::string data_directory,
        std::string output_directory,
        const std::string& configuration_directory,
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

  std::cout << "Trajectory ID: " << trajectory_id << "\n";

  TrajectoryBuilderInterface* trajectory_builder = mapBuilderViam.map_builder_->GetTrajectoryBuilder(trajectory_id);

  cartographer::io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  //std::vector<transform::Rigid3d> final_poses_non_optimized;
  std::vector<transform::Rigid3d> final_poses_non_optimized;

  std::cout << "Beginning to add data....\n";
  
  int j = 1;
  for (int i = 0; i < int(file_list.size()); i++ ) { // file_list.size()
    auto measurement = mapBuilderViam.GenerateSavedRangeMeasurements(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
        PaintMap(mapBuilderViam.map_builder_, output_directory, j++);
    }
  }

  mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilderViam.map_builder_, output_directory, 0);

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

  std::string data_directory = "/home/jeremyhyde-viam/data_slam_1";
  std::string output_directory = "/home/jeremyhyde-viam/pictures";

  cartographer::mapping::Run("Global2D", data_directory, output_directory, FLAGS_configuration_directory, FLAGS_configuration_basename);

  return 1;
}
