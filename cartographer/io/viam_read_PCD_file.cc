#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>  // std::ifstream
#include <string>
#include <boost/filesystem.hpp>
#include "viam_read_PCD_file.h"
#include <stdio.h>
#include <ctime>


namespace cartographer {
namespace io {

namespace fs = boost::filesystem;

sensor::TimedPointCloudData ReadFile::timedPointCloudDataFromPCDBuilder (std::string file_path, std::string initial_filename){

  sensor::TimedPointCloudData timedPCD;
  sensor::TimedPointCloud ranges;
  //std::vector<float> intensities;
  Eigen::Vector3f origin(0.0, 0.0, 0.0);
  
 //Open the point cloud file
 std::cout << "Accessing file " << file_path << " ... \n"; 

 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 auto err = pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_path, *cloud);

  if (err == -1) 
  {
    return timedPCD;
  }

   // KAT NOTE: The file name format for the pcd files is assumed to be, e.g.:
  // rplidar_data_2022-02-05T01_00_20.9874.pcd
  
  int start_pos = initial_filename.find("T") + 1;
  int len_pos = initial_filename.find(".pcd") - initial_filename.find("T") - 1;
  std::string initial_file = initial_filename.substr(start_pos, len_pos);
  std::string next_file = file_path.substr(start_pos, len_pos);

  //std::cout << "str 01: " << initial_file << std::endl;
  //std::cout << "str 02: " << next_file << std::endl;
  std::string::size_type sz;

  // Hour
 // std::cout << "str 02: " << next_file.substr(0,2) << std::endl;
  float hour_f = std::stof(next_file.substr(0,2), &sz);
  float hour_i = std::stof(initial_file.substr(0,2), &sz);

  // Minute
  //std::cout << "str 02: " << next_file.substr(3,2) << std::endl;
  float min_f = std::stof(next_file.substr(3,2), &sz);
  float min_i = std::stof(initial_file.substr(3,2), &sz);

  // Second
  //std::cout << "str 02: " << next_file.substr(6) << std::endl;
  float sec_f = std::stof(next_file.substr(6), &sz);
  float sec_i = std::stof(initial_file.substr(6), &sz);

  float total_time_final = 3600 * hour_f + 60 * min_f + sec_f;
  float total_time_initial = 3600 * hour_i + 60 * min_i + sec_i;

  float time_delta = 3600*(hour_f-hour_i) + 60*(min_f-min_i) + (sec_f - sec_i);

  //std::cout << "Final - HR: " << hour_i << " MIN: " << min_i << " SEC: " << sec_i << " | " << time_delta << std::endl;
  //std::cout << "Final - HR: " << hour_f << " MIN: " << min_f << " SEC: " << sec_f << " | " << time_delta << std::endl;
  std::cout << "------------ FILE DATA -------------\n";
  std::cout << "Loaded " << cloud->width * cloud->height << " data points \n";
  std::cout << "Size " << cloud->points.size() << "\n";
  std::cout << "TD " << time_delta << "\n";
  std::cout << "------------------------------------\n";

  for (size_t i = 0; i < cloud->points.size(); ++i) {

    sensor::TimedRangefinderPoint TimedRP;
    TimedRP.position = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    TimedRP.time = 0 - i*0.0001;
    
    ranges.push_back(TimedRP);
  }

  // std::cout << "PRINT TIME1 = " << double(time_delta) << std::endl;
  // std::cout << "PRINT TIME = " << (double(time_delta + 0.001 * time_delta_millis)) << std::endl;
  //timedPCD.time = cartographer::common::FromUniversal(int64(10000 + time_delta_millis + 1000*time_delta));

  //timedPCD.time = cartographer::common::FromUniversal(123) + cartographer::common::FromMilliseconds(double(time_delta));
  timedPCD.time = cartographer::common::FromUniversal(123) + cartographer::common::FromSeconds(double(time_delta));

  timedPCD.origin = Eigen::Vector3f::Zero();
  timedPCD.ranges = ranges;

  return timedPCD;
}

std::vector<std::string> ReadFile::listFilesInDirectory(std::string data_directory)
{
    std::vector<std::string> file_paths;

    for (const auto & entry : fs::directory_iterator(data_directory)) {
        file_paths.push_back((entry.path()).string());
    }

    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

int ReadFile::removeFile (std::string file_path)
{
  if( remove(file_path.c_str()) != 0 ) {
    std::cout<< "Error removing file\n"; 
    return 0;
  }
  return 1;
}

}  // namespace io
}  // namespace cartographer
