#include "cartographer/mapping/viam_map_builder.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/metrics/register.h"
#include <string>

namespace cartographer {
namespace mapping {

using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;

const SensorId kRangeSensorId{SensorId::SensorType::RANGE, "range"};
const SensorId kIMUSensorId{SensorId::SensorType::IMU, "imu"};


void PrintState(MapBuilderViam* mapBuilderViam, int trajectory_id, std::vector<std::string> times, std::vector<transform::Rigid3d> final_poses) {
  const auto trajectory_nodes = mapBuilderViam->map_builder_->pose_graph()->GetTrajectoryNodes();
  const auto submap_data = mapBuilderViam->map_builder_->pose_graph()->GetAllSubmapData();

  const transform::Rigid3d final_pose = mapBuilderViam->map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) * mapBuilderViam->local_slam_result_poses_.back();

  sleep(5);
  std::cout << "------------------ FINAL POSES ------------------\n";
  for (int i = 0; i < int(final_poses.size()); i++ ) { // file_list.size()
    //std::cout << "Pose: "  << final_poses_non_optimized[i] << "std::endl";
    std::cout << times[i] << "\t | \t" << final_poses[i] << std::endl;

  }
  std::cout << "-------------------------------------------------\n";
  std::cout << "Final Pose: " << final_pose << std::endl;
  std::cout << "-------------------------------------------------\n";
  //std::cout << "Size of localSLAMResultPose = " << mapBuilderViam->local_slam_result_poses_.size() << std::endl;

  return;
}



void Run(std::string mode, std::string data_directory, const std::string& configuration_directory,
         const std::string& configuration_basename) {

  MapBuilderViam mapBuilderViam;

  // Add configs
  mapBuilderViam.SetUp(configuration_directory, configuration_basename);
  mapBuilderViam.SetOptionsEnableGlobalOptimization();
  mapBuilderViam.BuildMapBuilder();

  //Run_loop(&mapBuilderViam, data_directory);

  // Get file directory to iterate though
  cartographer::io::ReadFile read_file;
  std::vector<std::string> file_list = read_file.listFilesInDirectory(data_directory);
  std::string initial_file = file_list[0];

  std::vector<std::string> times;
  std::vector<transform::Rigid3d> final_poses;

  int trajectory_id = mapBuilderViam.map_builder_->AddTrajectoryBuilder({kRangeSensorId}, mapBuilderViam.trajectory_builder_options_,
      mapBuilderViam.GetLocalSlamResultCallback());
  TrajectoryBuilderInterface* trajectory_builder = mapBuilderViam.map_builder_->GetTrajectoryBuilder(trajectory_id);
  

  // Run loops
  for (int i = 0; i < int(file_list.size()); i++ ) { // file_list.size()

    // TrajectoryBuilderInterface* trajectory_builder = mapBuilderViam->map_builder_->GetTrajectoryBuilder(trajectory_id);
    // int trajectory_id = mapBuilderViam->map_builder_->AddTrajectoryBuilder({kRangeSensorId}, mapBuilderViam->trajectory_builder_options_,
    //   mapBuilderViam->GetLocalSlamResultCallback());


    auto measurement = mapBuilderViam.GenerateSavedRangeMeasurements(data_directory, initial_file, i);

    if (measurement.ranges.size() > 0) {
      trajectory_builder->AddSensorData(kRangeSensorId.id, measurement);
    }

    if (i > 0) {
        try {
          //mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
          mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();
          const transform::Rigid3d final_pose_opt = mapBuilderViam.map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id) * mapBuilderViam.local_slam_result_poses_.back();
          final_poses.push_back(final_pose_opt);
          std::cout <<"Final Pose Found\n";
        } catch (...) {
          std::cout <<"Final Pose NOT Found\n";
        }
    }

    std::string curr_file = file_list[i];
    times.push_back(curr_file.substr(curr_file.find("T") + 1, curr_file.find(".pcd") - curr_file.find("T") - 1));
    // mapBuilderViam->map_builder_->FinishTrajectory(trajectory_id);
    // mapBuilderViam->map_builder_->pose_graph()->RunFinalOptimization();

    // const std::string filename = "temp-SaveLoadState.pbstream";
    // io::ProtoStreamWriter writer(filename);
    // map_builder_->SerializeState(/*include_unfinished_submaps=*/true, &writer);
    // writer.Close();
  }

  mapBuilderViam.map_builder_->FinishTrajectory(trajectory_id);
  //mapBuilderViam.map_builder_->pose_graph()->RunFinalOptimization();

  PrintState(&mapBuilderViam, trajectory_id, times, final_poses);
    
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
    data_directory = "/home/jeremyhyde-viam/data_10ms_loop";
    std::cout << "No data directory specified, using default: " << data_directory << std::endl;
  }

  cartographer::mapping::Run(mode, data_directory, FLAGS_configuration_directory, FLAGS_configuration_basename);

  return 1;
}
