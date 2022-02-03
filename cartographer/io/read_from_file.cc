#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <liblas/liblas.hpp>
#include <fstream>  // std::ifstream
#include <string>
#include <boost/filesystem.hpp>
#include "read_from_file.h"
#include <stdio.h>
#include <ctime>


namespace cartographer {
namespace io {

namespace fs = boost::filesystem;

sensor::TimedPointCloud ReadFile::timedPointCloudFromLASBuilder (std::string file_path){

  sensor::TimedPointCloudData timedPCD;
  sensor::TimedPointCloud ranges;

  std::ifstream ifs;
  ifs.open(file_path, std::ios::in |     std::ios::binary);

  std::cout << "Accessing file " << file_path << " ... \n"; 
  return ranges;
  liblas::ReaderFactory f; 
  liblas::Reader reader = f.CreateWithStream(ifs);

  std::vector<float> intensities;

  Eigen::Vector3f origin(0.0, 0.0, 0.0);

  std::cout << "Reading point data... \n";

  float t = 0;
  while (reader.ReadNextPoint()){
    liblas::Point const& p = reader.GetPoint();

    sensor::TimedRangefinderPoint TimedRP;
    TimedRP.position = Eigen::Vector3f(p.GetX(),p.GetY(),p.GetZ());
    TimedRP.time = t;
    
    ranges.push_back(TimedRP);
    intensities.push_back(p.GetIntensity());

    t += 0.1;
  }

  timedPCD.origin = origin;
  timedPCD.ranges = ranges;
  timedPCD.intensities = intensities;

  std::cout << "TimedPointCloudData# packet created! \n";
  std::cout << timedPCD.ranges.size() << " points registered \n";
  
  return ranges;
  //return timedPCD;
}

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

  std::string initial_file = initial_filename.substr(initial_filename.find("data_") + 5, 18);
  std::string next_file = file_path.substr(file_path.find("data_") + 5, 18);

  struct tm initial_time,final_time;
  strptime(initial_file.c_str(), "%Y%m%dT%H_%M_%SZ", &initial_time);
  strptime(next_file.c_str(), "%Y%m%dT%H_%M_%SZ", &final_time);
  initial_time.tm_isdst = -1;
  final_time.tm_isdst = -1;
  float time_delta = std::difftime(std::mktime(&final_time), std::mktime(&initial_time));

  std::cout << "Loaded " << cloud->width * cloud->height << " data points \n";
  std::cout << "Size " << cloud->points.size() << "\n";
  std::cout << "TD " << time_delta << "\n";

  int j = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i) {

    sensor::TimedRangefinderPoint TimedRP;
    TimedRP.position = Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    TimedRP.time = 0 - i*0.001;
    
    ranges.push_back(TimedRP);
  }

  timedPCD.time = cartographer::common::FromUniversal(123) + cartographer::common::FromSeconds(double(time_delta));
  timedPCD.origin = Eigen::Vector3f::Zero();
  timedPCD.ranges = ranges;

  return timedPCD;
}

std::vector<std::string> ReadFile::listFilesInDirectory()
{
    std::string path = "/home/jeremyhyde-viam/data";
    std::vector<std::string> file_paths;

    for (const auto & entry : fs::directory_iterator(path)) {
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
