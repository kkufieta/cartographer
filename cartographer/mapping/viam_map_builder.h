#ifndef CARTOGRAPHER_MAP_BUILDER_VIAM_H_
#define CARTOGRAPHER_MAP_BUILDER_VIAM_H_


#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/viam_map_builder.h"

#include "cartographer/common/config.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/viam_read_PCD_file.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/testing/test_helpers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"


namespace cartographer {
namespace mapping {


class MapBuilderViam {
  public:
  void SetUp(std::string configuration_directory, std::string configuration_basename);

  void BuildMapBuilder();

  void SetOptionsTo3D();
  void SetOptionsToTSDF2D();
  void SetOptionsEnableGlobalOptimization();

  MapBuilderInterface::LocalSlamResultCallback GetLocalSlamResultCallback();

  cartographer::sensor::TimedPointCloudData GenerateSavedRangeMeasurements(std::string data_directory, std::string initial_filename, int i);
  cartographer::sensor::TimedPointCloudData GenerateSaved2DRangeMeasurements(std::string initial_filename, int i, std::string data_directory);
  
  cartographer::sensor::TimedPointCloudData GetDataFromFile(std::string data_directory, std::string initial_filename, int i);


  std::unique_ptr<MapBuilderInterface> map_builder_;
  proto::MapBuilderOptions map_builder_options_;
  proto::TrajectoryBuilderOptions trajectory_builder_options_;
  std::vector<::cartographer::transform::Rigid3d> local_slam_result_poses_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_READ_FROM_FILE_H_