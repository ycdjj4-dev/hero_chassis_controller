// 节点主类实现。
#include "hero_chassis_controller/RosPackageTemplate.hpp"

#include <string>

namespace hero_chassis_controller {
    RosPackageTemplate::RosPackageTemplate(ros::NodeHandle &node_handle)
        : node_handle_(node_handle) {
        if (!ReadParameters()) {
            ROS_ERROR("Could not read parameters.");
            ros::requestShutdown();
        }
        subscriber_ = node_handle_.subscribe(subscriber_topic_, 1,
                                             &RosPackageTemplate::TopicCallback,
                                             this);
        service_server_ = node_handle_.advertiseService(
            "get_average", &RosPackageTemplate::ServiceCallback, this);
        ROS_INFO("Successfully launched node.");
    }

    RosPackageTemplate::~RosPackageTemplate() = default;

    bool RosPackageTemplate::ReadParameters() {
        if (!node_handle_.getParam("subscriber_topic", subscriber_topic_)) {
            return false;
        }
        return true;
    }

    void RosPackageTemplate::TopicCallback(const geometry_msgs::Twist &message) {
        // 提取底盘速度。
        const double vx = message.linear.x;
        const double vy = message.linear.y;
        const double wz = message.angular.z;

        // 组成底盘速度向量。
        const Eigen::Vector3d chassis_velocity(vx, vy, wz);

        // 逆运动学：计算四个车轮的目标角速度。
        const Eigen::Vector4d wheel_velocity =
                algorithm_.inverseKinematics(chassis_velocity);

        // 打印计算结果，方便验证。
        ROS_INFO_STREAM_THROTTLE(1.0,
                                 "Wheel velocities: " << wheel_velocity.transpose());
    }

    bool RosPackageTemplate::ServiceCallback(
        std_srvs::Trigger::Request & /*request*/,
        std_srvs::Trigger::Response &response) {
        response.success = true;
        response.message =
                "The average is " + std::to_string(algorithm_.getAverage());
        return true;
    }
} // namespace hero_chassis_controller
