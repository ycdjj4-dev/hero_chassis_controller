#include "hero_chassis_controller/RosPackageTemplate.hpp"

// STD
#include <string>

namespace hero_chassis_controller {

RosPackageTemplate::RosPackageTemplate(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle) {
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &RosPackageTemplate::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &RosPackageTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosPackageTemplate::~RosPackageTemplate() {
}

bool RosPackageTemplate::readParameters() {
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

  void RosPackageTemplate::topicCallback(
      const geometry_msgs::Twist &message) {

  // 提取底盘速度
  double vx = message.linear.x;
  double vy = message.linear.y;
  double wz = message.angular.z;

  // 组成底盘速度向量
  Eigen::Vector3d chassis_velocity(vx, vy, wz);

  // 逆运动学：计算四个车轮的目标角速度
  Eigen::Vector4d wheel_velocity =
      algorithm_.inverseKinematics(chassis_velocity);

  // 暂时打印计算结果，方便验证
  ROS_INFO_STREAM_THROTTLE(
      1.0, "Wheel velocities: " << wheel_velocity.transpose());
}

bool RosPackageTemplate::serviceCallback(std_srvs::Trigger::Request &request,
                                         std_srvs::Trigger::Response &response) {
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
