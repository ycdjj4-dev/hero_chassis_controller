// 麦轮底盘控制器：订阅 /cmd_vel，经逆运动学与四轮 PID 输出关节力矩，
// 并发布里程计 (odom) 与 TF。
#ifndef HERO_CHASSIS_CONTROLLER_HEROCHASSISCONTROLLER_HPP_
#define HERO_CHASSIS_CONTROLLER_HEROCHASSISCONTROLLER_HPP_

#include <memory>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "hero_chassis_controller/Algorithm.hpp"

namespace hero_chassis_controller {
    // 麦轮底盘控制器 (ros_control 插件)。
    class HeroChassisController
            : public controller_interface::Controller<
                hardware_interface::EffortJointInterface> {
    public:
        // 初始化控制器：获取关节句柄并加载四轮 PID 参数。
        bool init(hardware_interface::EffortJointInterface *hw,
                  ros::NodeHandle & /*root_nh*/,
                  ros::NodeHandle &controller_nh) override;

        // 控制器启动：重置 PID 与力矩输出。
        void starting(const ros::Time &time) override;

        // 控制器周期更新。
        void update(const ros::Time &time, const ros::Duration &period) override;

        // 控制器停止。
        void stopping(const ros::Time &time) override;

    private:
        // 接收底盘速度指令并写入实时缓冲区。
        void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);

        // 四个车轮的关节句柄。
        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;

        // 四个车轮的速度 PID。
        control_toolbox::Pid pid_front_left_;
        control_toolbox::Pid pid_front_right_;
        control_toolbox::Pid pid_back_left_;
        control_toolbox::Pid pid_back_right_;

        // 速度指令订阅器。
        ros::Subscriber cmd_vel_subscriber_;

        // 实时数据缓冲区。
        realtime_tools::RealtimeBuffer<geometry_msgs::TwistStamped> command_buffer_;

        // 里程计发布器。
        std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> >
        odom_publisher_;

        // TF 发布器。
        std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> >
        tf_publisher_;

        // 运动学算法。
        Algorithm algorithm_;

        // 速度指令超时时间 (s)。
        double cmd_vel_timeout_{0.5};

        // 摩擦前馈增益 (N·m·s/rad)。
        double feedforward_gain_{0.05};

        // 速度指令坐标系模式: "chassis" 底盘坐标系 / "world" 全局坐标系。
        std::string velocity_mode_{"chassis"};

        // 里程计累计位姿 (m, m, rad)。
        double odom_x_{0.0};
        double odom_y_{0.0};
        double odom_yaw_{0.0};

        // 控制器启动时间。
        ros::Time controller_start_time_;
    };
} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_HEROCHASSISCONTROLLER_HPP_
