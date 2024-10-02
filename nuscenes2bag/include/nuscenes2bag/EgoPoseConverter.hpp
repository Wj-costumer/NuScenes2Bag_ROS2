#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "nuscenes2bag/utils.hpp"
#include <nuscenes2bag/MetaDataTypes.hpp>

namespace nuscenes2bag {

nav_msgs::msg::Odometry
egoPoseInfo2OdometryMsg(const EgoPoseInfo& egoPoseInfo);

geometry_msgs::msg::TransformStamped
egoPoseInfo2TransformStamped(const EgoPoseInfo& egoPoseInfo);

}
