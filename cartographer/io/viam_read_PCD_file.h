#ifndef CARTOGRAPHER_READ_FROM_FILE_H_
#define CARTOGRAPHER_READ_FROM_FILE_H_

#include <chrono>
#include <ostream>
#include <ratio>
#include <inttypes.h>
#include <string>
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace io {


class ReadFile {
 public:
 std::vector<std::string> listFilesInDirectory(std::string data_directory);
  sensor::TimedPointCloudData timedPointCloudDataFromPCDBuilder(std::string file_path, std::string initial_filename);
  int removeFile(std::string);

};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_READ_LAS_FILE_H_