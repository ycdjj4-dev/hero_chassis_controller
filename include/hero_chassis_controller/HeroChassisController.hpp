#pragma once

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// ROS Control
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// PID
#include <control_toolbox/pid.h>

// Realtime
#include <realtime_tools/realtime_buffer.h>

// Algorithm
#include "hero_chassis_controller/Algorithm.hpp"

#include <geometry_msgs/TwistStamped.h>


namespace hero_chassis_controller {

    class HeroChassisController
        : public controller_interface::Controller<
              hardware_interface::EffortJointInterface> {

    public:
        // 初始化控制器
        bool init(
            hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

        // 控制器启动
        void starting(const ros::Time& time) override;

        // 控制器循环更新
        void update(
            const ros::Time& time,
            const ros::Duration& period) override;

        // 控制器停止
        void stopping(const ros::Time& time) override;

    private:

        // 接收底盘速度指令
        void cmdVelCallback(
            const geometry_msgs::TwistConstPtr& msg);

        // 四个车轮的关节句柄
        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;

        // 四个车轮的 PID 控制器
        control_toolbox::Pid pid_front_left_;
        control_toolbox::Pid pid_front_right_;
        control_toolbox::Pid pid_back_left_;
        control_toolbox::Pid pid_back_right_;

        // 接收速度指令
        ros::Subscriber cmd_vel_subscriber_;

        // 实时数据缓冲区
        realtime_tools::RealtimeBuffer<geometry_msgs::TwistStamped> command_buffer_;

        // 运动学算法
        Algorithm algorithm_;

        // 速度指令超时时间，默认 0.5 秒
        double cmd_vel_timeout_{0.5};

        // 摩擦前馈增益 (N·m·s/rad)，默认 0.05
        double feedforward_gain_{0.05};

        // 控制器启动时间
        ros::Time controller_start_time_;
    };

} // namespace hero_chassis_controller