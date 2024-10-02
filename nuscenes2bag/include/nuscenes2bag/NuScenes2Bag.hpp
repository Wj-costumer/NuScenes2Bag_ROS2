#pragma once

#include <rosbag2_cpp/writer.hpp>

#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/Filesystem.hpp"

#include <boost/optional.hpp>

class Demo
{
  public:
    void test(int a);
};

namespace nuscenes2bag {
  class NuScenes2Bag {
    public:
      void convertDirectory(fs::path inDatasetPath,
                            std::string version,
                            fs::path outputRosbagPath,
                            int32_t threadNumber,
                            boost::optional<int32_t> sceneNumberOpt
                            );

    private:
      std::string inDatasetPathString;
};
}