#pragma once

#include "sensor_msgs/msg/image.h"
#include "nuscenes2bag/Filesystem.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/optional.hpp>

namespace nuscenes2bag {

boost::optional<sensor_msgs::msg::Image> readImageFile(const fs::path& filePath) noexcept;

}
