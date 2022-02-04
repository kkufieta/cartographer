#include "cartographer/mapping/viam_map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include <string>

namespace cartographer {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};



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

  std::cout << "Trajectory ID: " << trajectory_id << "\n";

  TrajectoryBuilderInterface* trajectory_builder = mapBuilderViam.map_builder_->GetTrajectoryBuilder(trajectory_id);

  float i = 0;

  cartographer::io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  std::cout << "Beginning to add data....\n";
  
  for (int i = 0; i < int(file_list.size()); i++ ) { // file_list.size()
    auto measurement = mapBuilderViam.GenerateSavedRangeMeasurements(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
        trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    }
  }

  mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
  mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();

  if (mode == "Global2D") {
      const auto trajectory_nodes = mapBuilderViam.map_builder_->pose_graph()->GetTrajectoryNodes();
      const auto submap_data = mapBuilderViam.map_builder_->pose_graph()->GetAllSubmapData();

      const transform::Rigid3d final_pose =
          mapBuilderViam.map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) * mapBuilderViam.local_slam_result_poses_.back();

      std::cout << "Time: " << i << "\t | Final Pose: " << final_pose << std::endl;
  }

  std::cout << "Size of localSLAMREsultPose = " << mapBuilderViam.local_slam_result_poses_.size() << std::endl;
  // TODO: add map extractor
  //PcdWritingPointsProcessor pointProcessor;
  //pointProcessor.Process();
  return;
}

}  // namespace mapping
}  // namespace cartographer

int main(int argc, char** argv) {

  std::string FLAGS_configuration_directory = "../configuration_files";
  std::string FLAGS_configuration_basename = "viam_rplidar.lua";

  std::string mode = "Global2D";

  std::string data_directory;
  if (argc >= 2) {
    data_directory = argv[1];
  }
  else {
    data_directory = "/home/jeremyhyde-viam/data";
    std::cout << "No data directory specified, using default: " << data_directory << std::endl;
  }

  cartographer::mapping::Run(mode, data_directory, FLAGS_configuration_directory, FLAGS_configuration_basename);

  return 1;
}
