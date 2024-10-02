#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "nuscenes2bag/Filesystem.hpp"
#include <boost/optional.hpp>

namespace nuscenes2bag {

boost::optional<sensor_msgs::msg::PointCloud2> readLidarFileXYZIR(const fs::path& filePath);

}
