#include "nuscenes2bag/utils.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "nuscenes2bag/utils.hpp"
#include <nuscenes2bag/MetaDataTypes.hpp>

namespace nuscenes2bag {

nav_msgs::msg::Odometry
egoPoseInfo2OdometryMsg(const EgoPoseInfo& egoPoseInfo)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stampUs2RosTime(egoPoseInfo.timeStamp);
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  assignArray2Vector3(msg.pose.pose.position, egoPoseInfo.translation);
  assignArray2Quaternion(msg.pose.pose.orientation, egoPoseInfo.rotation);

  return msg;
}

geometry_msgs::msg::TransformStamped
egoPoseInfo2TransformStamped(const EgoPoseInfo& egoPoseInfo)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stampUs2RosTime(egoPoseInfo.timeStamp);
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  assignArray2Vector3(msg.transform.translation, egoPoseInfo.translation);
  assignArray2Quaternion(msg.transform.rotation, egoPoseInfo.rotation);

  return msg;
}

}