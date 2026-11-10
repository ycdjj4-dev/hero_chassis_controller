// 节点主类：负责 ROS 接口（订阅 Twist、提供取平均服务）。
#ifndef HERO_CHASSIS_CONTROLLER_ROSPACKAGETEMPLATE_HPP_
#define HERO_CHASSIS_CONTROLLER_ROSPACKAGETEMPLATE_HPP_

#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

#include "hero_chassis_controller/Algorithm.hpp"

namespace hero_chassis_controller {
    // 节点主类：负责 ROS 接口（订阅 Twist、提供取平均服务）。
    class RosPackageTemplate {
    public:
        // 构造函数。
        // @param node_handle ROS 节点句柄。
        explicit RosPackageTemplate(ros::NodeHandle &node_handle);

        virtual ~RosPackageTemplate();

    private:
        // 读取并校验 ROS 参数。
        // @return 读取成功返回 true。
        bool ReadParameters();

        // ROS 话题回调。
        // @param message 收到的消息。
        void TopicCallback(const geometry_msgs::Twist &message);

        // ROS 服务回调。
        // @param request 服务请求。
        // @param response 服务响应。
        // @return 处理成功返回 true。
        bool ServiceCallback(std_srvs::Trigger::Request &request,
                             std_srvs::Trigger::Response &response);

        // ROS 节点句柄。
        ros::NodeHandle &node_handle_;

        // ROS 话题订阅器。
        ros::Subscriber subscriber_;

        // 订阅的话题名。
        std::string subscriber_topic_;

        // ROS 服务服务器。
        ros::ServiceServer service_server_;

        // 算法计算对象。
        Algorithm algorithm_;
    };
} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_ROSPACKAGETEMPLATE_HPP_
