#include "nuscenes2bag/SceneConverter.hpp"
#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/utils.hpp"
#include <boost/optional.hpp>
#include "nuscenes2bag/EgoPoseConverter.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverterXYZIR.hpp"
#include "nuscenes2bag/RadarDirectoryConverter.hpp"
#include <rclcpp/serialization.hpp>
#include <memory>
#include <array>
#include <iostream>
#include <regex>
#include <string>
#include "sensor_msgs/msg/image.h"
#include "sensor_msgs/msg/point_cloud2.h"


using namespace std;

namespace nuscenes2bag {

SceneConverter::SceneConverter(const MetaDataProvider& metaDataProvider)
  : metaDataProvider(metaDataProvider)
{}

boost::optional<SampleType>
getSampleType(const std::string& filename)
{
  std::array<std::pair<const char*, SampleType>, 3> pairs = {
    { { "CAM", SampleType::CAMERA },
      { "RADAR", SampleType::RADAR },
      { "LIDAR", SampleType::LIDAR } }
  };
  for (const auto& strAndSampleType : pairs) {
    const auto& str = strAndSampleType.first;
    const auto& sampleType = strAndSampleType.second;
    if (filename.find(str) != string::npos) {
      return boost::optional<SampleType>(sampleType);
    }
  }
  cout << "Unknown file " << filename << endl;
  return boost::none;
}

static const std::regex TOPIC_REGEX = std::regex(".*__([A-Z_]+)__.*");

void
SceneConverter::submit(const Token& sceneToken, FileProgress& fileProgress)
{

  boost::optional<SceneInfo> sceneInfoOpt =
    metaDataProvider.getSceneInfo(sceneToken);
  // if(!sceneInfoOpt) {
  //     // cout << "SceneInfo for " << sceneToken << " not found!" << endl;
  //     return;
  // }
  assert(sceneInfoOpt);
  SceneInfo& sceneInfo = sceneInfoOpt.value();

  sceneId = sceneInfo.sceneId;
  this->sceneToken = sceneToken;
  sampleDatas = metaDataProvider.getSceneSampleData(sceneToken);
  egoPoseInfos = metaDataProvider.getEgoPoseInfo(sceneToken);

  fileProgress.addToProcess(sampleDatas.size());
}

void
SceneConverter::run(const fs::path& inPath,
                    const fs::path& outDirectoryPath,
                    FileProgress& fileProgress)
{

  std::string bagName =
    outDirectoryPath.string() + "/" + std::to_string(sceneId) + ".bag";

  std::unique_ptr<rosbag2_cpp::Writer> outBag;
  outBag = std::make_unique<rosbag2_cpp::Writer>();
  std::cout << "output: " << bagName << std::endl;
  outBag->open(bagName);

  auto sensorInfos = metaDataProvider.getSceneCalibratedSensorInfo(sceneToken);
  convertEgoPoseInfos(outBag, sensorInfos);
  convertSampleDatas(outBag, inPath, fileProgress);

  outBag->close();
}

void
SceneConverter::convertSampleDatas(std::unique_ptr<rosbag2_cpp::Writer>& outBag,
                                   const fs::path& inPath,
                                   FileProgress& fileProgress)
{
  for (const auto& sampleData : sampleDatas) {
    fs::path sampleFilePath = inPath / sampleData.fileName;

    boost::optional<SampleType> sampleTypeOpt =
      getSampleType(sampleFilePath.string());
    if (!sampleTypeOpt) {
      continue;
    }
    SampleType& sampleType = sampleTypeOpt.value();

    CalibratedSensorInfo calibratedSensorInfo =
      metaDataProvider.getCalibratedSensorInfo(
        sampleData.calibratedSensorToken);
    CalibratedSensorName calibratedSensorName =
      metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
    const std::string sensorName = toLower(calibratedSensorName.name);

    if (sampleType == SampleType::CAMERA) {
      const std::string topicName = sensorName + "/raw";
      sensor_msgs::msg::Image msg = readImageFile(sampleFilePath).value();
      rclcpp::Time stamp = stampUs2RosTime(sampleData.timeStamp);
      std::string dataType = "sensor_msgs::msg::Image";
      outBag->create_topic(
        {
          topicName,
          "sensor_msgs/msg/Image",
          rmw_get_serialization_format(),
          ""
        }
      );
      outBag->write(msg, topicName, stamp);
    } else if (sampleType == SampleType::LIDAR) {
      std::string topicName = sensorName;

      // PointCloud format:
      sensor_msgs::msg::PointCloud2 msg = readLidarFile(sampleFilePath).value(); // x,y,z,intensity
      // auto msg = readLidarFileXYZIR(sampleFilePath); // x,y,z,intensity,ring

      std::string dataType = "sensor_msgs::msg::PointCloud2";
      rclcpp::Time stamp = stampUs2RosTime(sampleData.timeStamp);
      
      outBag->create_topic(
        {
          topicName,
          "sensor_msgs/msg/PointCloud2",
          rmw_get_serialization_format(),
          ""
        }
      );
      outBag->write(msg, topicName, stamp);
    } else if (sampleType == SampleType::RADAR) {

      std::cout << "write radar data" << std::endl; // just a test

      std::string topicName = sensorName;
      message_interfaces::msg::RadarObjects msg = readRadarFile(sampleFilePath).value();

      std::string dataType = "message_interfaces::msg::RadarObjects";

      outBag->create_topic(
        {
          topicName,
          "message_interfaces/msg/RadarObjects",
          rmw_get_serialization_format(),
          {},
        }
      );

      // just a test
      std::cout << sampleData.timeStamp << std::endl;

      rclcpp::Time stamp = stampUs2RosTime(sampleData.timeStamp);
      outBag->write(msg, topicName, stamp);
    } else {
      cout << "Unknown sample type" << endl;
    }

    fileProgress.addToProcessed(1);
  }
}

geometry_msgs::msg::TransformStamped
makeTransform(const char* frame_id,
              const char* child_frame_id,
              const double* translation,
              const double* rotation,
              rclcpp::Time stamp = rclcpp::Time(0))
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  assignArray2Vector3(msg.transform.translation, translation);
  assignArray2Quaternion(msg.transform.rotation, rotation);
  return msg;
}

geometry_msgs::msg::TransformStamped
makeIdentityTransform(const char* frame_id,
                      const char* child_frame_id,
                      rclcpp::Time stamp = rclcpp::Time(0))
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  msg.transform.rotation.w = 1;
  return msg;
}

void SceneConverter::convertEgoPoseInfos(
  std::unique_ptr<rosbag2_cpp::Writer>& outBag,
  const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfos)
{

  std::vector<geometry_msgs::msg::TransformStamped> constantTransforms;
  for (const auto& calibratedSensorInfo : calibratedSensorInfos) {
    auto sensorTransform =
      makeTransform("base_link",
                    toLower(calibratedSensorInfo.name.name).c_str(),
                    calibratedSensorInfo.info.translation,
                    calibratedSensorInfo.info.rotation);
    constantTransforms.push_back(sensorTransform);
  }
  geometry_msgs::msg::TransformStamped tfMap2Odom =
    makeIdentityTransform("map", "odom");
  constantTransforms.push_back(tfMap2Odom);

  const std::string odomTopic = "/odom";
  for (const auto& egoPose : egoPoseInfos) {
    // write odom
    nav_msgs::msg::Odometry odomMsg = egoPoseInfo2OdometryMsg(egoPose);
    outBag->create_topic(
      {
        odomTopic,
        "nav_msgs/msg/Odometry",
        rmw_get_serialization_format(),
        ""
      }
    );
    outBag->write(odomMsg, odomTopic, odomMsg.header.stamp); 
    // write TFs
    geometry_msgs::msg::TransformStamped tfOdom2Base =
      egoPoseInfo2TransformStamped(egoPose);
    tf2_msgs::msg::TFMessage tfMsg;
    tfMsg.transforms.push_back(tfOdom2Base);
    for (const auto& constantTransform : constantTransforms) {
      auto constantTransformWithNewStamp = constantTransform;
      constantTransformWithNewStamp.header.stamp = odomMsg.header.stamp;
      tfMsg.transforms.push_back(constantTransformWithNewStamp);
    }
    outBag->create_topic(
      {
        "/tf",
        "tf2_msgs/msg/TFMessage",
        rmw_get_serialization_format(),
        ""
      }
    );
    outBag->write(tfMsg, "/tf", odomMsg.header.stamp);
  }
}

}