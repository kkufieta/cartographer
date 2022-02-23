#include "cartographer/mapping/viam_map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include <string>

#include "cartographer/io/submap_painter.h"
#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"
#include "glog/logging.h"

#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"

#include "cartographer/io/viam_draw_trajectories.h"


DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_mapping_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "mapping configuration file.");
DEFINE_string(configuration_localization_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "localization configuration file.");
DEFINE_string(data_directory, "",
              "Directory in which rplidar data is expected.");
DEFINE_string(output_directory, "",
              "Directory where map images are saved in.");
DEFINE_string(map_output_name, "",
              "Name of the file where we're saving the generated map.");
DEFINE_bool(mapping, false,
              "Name of the file where we're saving the generated map.");
DEFINE_bool(localization, false,
              "Name of the file where we're saving the generated map.");
DEFINE_int64(picture_print_interval, 1e8,
              "Frequency at which we want to print pictures while cartographer is running.");

namespace cartographer {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};


void PaintMap(std::unique_ptr<cartographer::mapping::MapBuilderInterface> & map_builder_, std::string output_directory, std::string appendix) {
  const double kPixelSize = 0.01;
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  std::map<cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;

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
        submap_slice.pose = submap_id_pose.data.pose;
        submap_slice.width = fetched_texture->width;
        submap_slice.height = fetched_texture->height;
        submap_slice.slice_pose = fetched_texture->slice_pose;
        submap_slice.resolution = fetched_texture->resolution;
        submap_slice.cairo_data.clear();

        // Paint map
        // if (submap_id_pose.id.trajectory_id == 0) {
          submap_slice.surface = ::cartographer::io::DrawTexture(
            fetched_texture->pixels.intensity, fetched_texture->pixels.alpha, fetched_texture->width,
            fetched_texture->height, &submap_slice.cairo_data);
        // } 
        // Paint trajectory, filtered to paint only trajectory with id == 0
        const auto trajectory_nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
        const cartographer::io::FloatColor color = {{1.f, 0.f, 0.f}};
        submap_slice.surface = cartographer::io::DrawTrajectoryNodes(trajectory_nodes, submap_slice.resolution, submap_slice.slice_pose, 
                                            color, submap_slice.surface.get());
      }

    cartographer::io::PaintSubmapSlicesResult painted_slices =
        PaintSubmapSlices(submap_slices, kPixelSize);
    auto image = cartographer::io::Image(std::move(painted_slices.surface));
    auto file = cartographer::io::StreamFileWriter(output_directory + "/map_" + appendix + ".png");
    image.WritePng(&file);
  }
}

void Run(const std::string& mode,
        const std::string& data_directory,
        const std::string& output_directory,
        const std::string& configuration_directory,
        const std::string& configuration_basename,
        const std::string& map_output_name,
        int picture_print_interval) {

  MapBuilderViam mapBuilderViam;

  // Add configs
  mapBuilderViam.SetUp(configuration_directory, configuration_basename);

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

  std::cout << "Beginning to add data....\n";
  
  for (int i = 0; i < int(file_list.size()); i++ ) {
    auto measurement = mapBuilderViam.GenerateSavedRangeMeasurements(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
        auto num_nodes = mapBuilderViam.map_builder_->pose_graph()->GetTrajectoryNodes().size();
        if (num_nodes < 3 || num_nodes % picture_print_interval == 0) {
          PaintMap(mapBuilderViam.map_builder_, output_directory, std::to_string(num_nodes));
        }
    }
  }

  // Save the map in a pbstream file
  const std::string map_file = "./" + map_output_name;
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  mapBuilderViam.map_builder_->SerializeStateToFile(true, map_file);

  mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilderViam.map_builder_, output_directory, "0");
  
  return;
}

void LoadMapAndRun(const std::string& mode,
        const std::string& data_directory,
        const std::string& output_directory,
        const std::string& configuration_directory,
        const std::string& configuration_basename,
        const std::string& map_output_name,
        int picture_print_interval) {

  MapBuilderViam mapBuilderViam;

  // Add configs
  mapBuilderViam.SetUp(configuration_directory, configuration_basename);

  // Build MapBuilder
  mapBuilderViam.BuildMapBuilder();
  const std::string map_file = "./" + map_output_name;
  std::map<int, int> mapping_of_trajectory_ids = mapBuilderViam.map_builder_->LoadStateFromFile(map_file, true);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  for(std::map<int, int>::const_iterator it = mapping_of_trajectory_ids.begin(); it != mapping_of_trajectory_ids.end(); ++it)
  {
      std::cout << "Trajectory ids mapping: " << it->first << " " << it->second << "\n";
  }

  // auto* options =
  //     mapBuilderViam.trajectory_builder_options_.mutable_pure_localization_trimmer();
  // options->set_max_submaps_to_keep(3);
  // mapBuilderViam.trajectory_builder_options_.set_pure_localization(true);

  // Build TrajectoryBuilder
  int trajectory_id = mapBuilderViam.map_builder_->AddTrajectoryBuilder(
      {kRangeSensorId}, mapBuilderViam.trajectory_builder_options_,
      mapBuilderViam.GetLocalSlamResultCallback());

  std::cout << "Trajectory ID: " << trajectory_id << "\n";

  TrajectoryBuilderInterface* trajectory_builder = mapBuilderViam.map_builder_->GetTrajectoryBuilder(trajectory_id);

  cartographer::io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  std::cout << "Beginning to add data....\n";
  PaintMap(mapBuilderViam.map_builder_, output_directory, "before_localization");
  
  for (int i = 500; i < int(file_list.size()); i++ ) {
    auto measurement = mapBuilderViam.GenerateSavedRangeMeasurements(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
        if (i < 3 || i % picture_print_interval == 0) {
          PaintMap(mapBuilderViam.map_builder_, output_directory, "localization_" + std::to_string(1 + i++));
        }
    }
  }

  // saved map after localization is finished
  const std::string map_file_2 = "./" + map_output_name + "after_localization";
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  mapBuilderViam.map_builder_->SerializeStateToFile(true, map_file_2);

  mapBuilderViam.map_builder_->FinishTrajectory(0);
  mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilderViam.map_builder_, output_directory, "after_localization_optimization");

  return;
}

void DrawSavedMap(const std::string& mode,
        const std::string& data_directory,
        const std::string& output_directory,
        const std::string& configuration_directory,
        const std::string& configuration_basename,
        const std::string& map_output_name) {

  MapBuilderViam mapBuilderViam;

  // Add configs
  mapBuilderViam.SetUp(configuration_directory, configuration_basename);

  // Build MapBuilder
  mapBuilderViam.BuildMapBuilder();
  const std::string map_file = "./" + map_output_name + "after_localization";
  std::map<int, int> mapping_of_trajectory_ids = mapBuilderViam.map_builder_->LoadStateFromFile(map_file, true);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  for(std::map<int, int>::const_iterator it = mapping_of_trajectory_ids.begin(); it != mapping_of_trajectory_ids.end(); ++it)
  {
      std::cout << "Trajectory ids mapping: " << it->first << " " << it->second << "\n";
  }

  PaintMap(mapBuilderViam.map_builder_, output_directory, map_output_name + "_saved_map_after_localization");

  mapBuilderViam.map_builder_->FinishTrajectory(0);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
  PaintMap(mapBuilderViam.map_builder_, output_directory, "optimized_" +  map_output_name + "_saved_map_after_localization");

  return;
}


}  // namespace mapping
}  // namespace cartographer

// Example of how to run this file: 
// .run_cart_main.sh
int main(int argc, char** argv) {
  google::InitGoogleLogging("XXX");
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = false;

  if (FLAGS_configuration_directory.empty()) {
    std::cout << "-configuration_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_configuration_localization_basename.empty()) {
    std::cout << "-configuration_localization_basename is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_configuration_mapping_basename.empty()) {
    std::cout << "-configuration_mapping_basename is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_data_directory.empty()) {
    std::cout << "-data_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_output_directory.empty()) {
    std::cout << "-output_directory is missing.\n";
    return EXIT_FAILURE;
  } else if (FLAGS_map_output_name.empty()) {
    std::cout << "-map_output_name is missing.\n";
    return EXIT_FAILURE;
  }

  google::SetCommandLineOption("GLOG_minloglevel", "2");

  std::string mode = "DON'T USE RIGHT NOW!!!!!!!";
  if (FLAGS_mapping == true) {
    std::cout << "Mapping!" << std::endl;
    cartographer::mapping::Run(mode,
      FLAGS_data_directory,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_mapping_basename,
      FLAGS_map_output_name,
      FLAGS_picture_print_interval);
  }

  if (FLAGS_localization == true) {
    std::cout << "Localizing!" << std::endl;
    cartographer::mapping::LoadMapAndRun(mode,
      FLAGS_data_directory,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_localization_basename,
      FLAGS_map_output_name,
      FLAGS_picture_print_interval);

    std::cout << "Drawing saved map!" << std::endl;
    cartographer::mapping::DrawSavedMap(mode,
      FLAGS_data_directory,
      FLAGS_output_directory,
      FLAGS_configuration_directory,
      FLAGS_configuration_localization_basename,
      FLAGS_map_output_name);
  }

  if (FLAGS_localization == false && FLAGS_mapping == false) {
    std::cout << "Not doing anything, both mapping & localization are turned off." << std::endl;
  }

  return 1;
}
