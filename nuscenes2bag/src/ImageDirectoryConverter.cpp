#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <thread>
#include <boost/optional.hpp>
#include "sensor_msgs/msg/image.hpp"

namespace nuscenes2bag {

boost::optional<sensor_msgs::msg::Image>
readImageFile(const fs::path& filePath) noexcept
{
  cv::Mat image;
  try {
    image = imread(filePath.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::msg::Image msg = 
      *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    return boost::optional<sensor_msgs::msg::Image>(msg);

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }

  return boost::none;
}

}