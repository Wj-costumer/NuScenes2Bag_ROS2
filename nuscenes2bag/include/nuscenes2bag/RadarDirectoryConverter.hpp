#pragma once
#include "message_interfaces/msg/radar_objects.hpp"
#include "nuscenes2bag/PclRadarObject.hpp"
#include "nuscenes2bag/Filesystem.hpp"
#include <boost/optional.hpp>

namespace nuscenes2bag {

boost::optional<message_interfaces::msg::RadarObjects> readRadarFile(const fs::path& filePath);

}