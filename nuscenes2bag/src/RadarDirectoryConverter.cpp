#include "nuscenes2bag/RadarDirectoryConverter.hpp"

// #include <pcl_ros/point_cloud.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include "message_interfaces/msg/radar_objects.hpp"

using namespace sensor_msgs;
using namespace std;
using namespace nuscenes2bag;

namespace nuscenes2bag {

boost::optional<message_interfaces::msg::RadarObjects>
readRadarFile(const fs::path& filePath)
{
  const auto fileName = filePath.string();
  pcl::PointCloud<PclRadarObject>::Ptr cloud(
    new pcl::PointCloud<PclRadarObject>);

  if (pcl::io::loadPCDFile<PclRadarObject>(fileName, *cloud) ==
      -1) //* load the file
  {
    std::string error = "Could not read ";
    error += fileName;
    cout << error << endl;
    // PCL_ERROR(error);

    return boost::none;
  }

  message_interfaces::msg::RadarObjects radarObjects;

  for (const auto& pclRadarObject : *cloud) {
    message_interfaces::msg::RadarObject obj;
    obj.pose.x = pclRadarObject.x;
    obj.pose.y = pclRadarObject.y;
    obj.pose.z = pclRadarObject.z;
    obj.dyn_prop = pclRadarObject.dyn_prop;
    obj.rcs = pclRadarObject.rcs;
    obj.vx = pclRadarObject.vx;
    obj.vy = pclRadarObject.vy;
    obj.vx_comp = pclRadarObject.vx_comp;
    obj.vy_comp = pclRadarObject.vy_comp;
    obj.is_quality_valid = pclRadarObject.is_quality_valid;
    obj.ambig_state = pclRadarObject.ambig_state;
    obj.x_rms = pclRadarObject.x_rms;
    obj.y_rms = pclRadarObject.y_rms;
    obj.invalid_state = pclRadarObject.invalid_state;
    obj.pdh0 = pclRadarObject.pdh0;
    obj.vx_rms = pclRadarObject.vx_rms;
    obj.vy_rms = pclRadarObject.vy_rms;
    radarObjects.objects.push_back(obj);
  }

  return boost::optional<message_interfaces::msg::RadarObjects>(radarObjects);
}

}