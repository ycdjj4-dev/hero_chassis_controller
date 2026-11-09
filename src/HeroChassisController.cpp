//
// Created by bingo on 2026/11/8.
//
#include "hero_chassis_controller/HeroChassisController.hpp"
#include <algorithm>

namespace hero_chassis_controller {

    bool HeroChassisController::init(
        hardware_interface::EffortJointInterface* hw,
        ros::NodeHandle&,
        ros::NodeHandle& controller_nh)
    {
        try {
            front_left_joint_ =
                hw->getHandle("left_front_wheel_joint");

            front_right_joint_ =
                hw->getHandle("right_front_wheel_joint");

            back_left_joint_ =
                hw->getHandle("left_back_wheel_joint");

            back_right_joint_ =
                hw->getHandle("right_back_wheel_joint");
            // 初始化左前轮 PID
            ros::NodeHandle pid_fl_nh(controller_nh, "pid_front_left");
            if (!pid_front_left_.init(pid_fl_nh)) {
                ROS_ERROR("Failed to initialize front left PID");
                return false;
            }

            // 初始化右前轮 PID
            ros::NodeHandle pid_fr_nh(controller_nh, "pid_front_right");
            if (!pid_front_right_.init(pid_fr_nh)) {
                ROS_ERROR("Failed to initialize front right PID");
                return false;
            }

            // 初始化左后轮 PID
            ros::NodeHandle pid_bl_nh(controller_nh, "pid_back_left");
            if (!pid_back_left_.init(pid_bl_nh)) {
                ROS_ERROR("Failed to initialize back left PID");
                return false;
            }

            // 初始化右后轮 PID
            ros::NodeHandle pid_br_nh(controller_nh, "pid_back_right");
            if (!pid_back_right_.init(pid_br_nh)) {
                ROS_ERROR("Failed to initialize back right PID");
                return false;
            }

        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM("Failed to get joint handles: " << e.what());
            return false;
        }
        geometry_msgs::TwistStamped initial_command;
        command_buffer_.initRT(initial_command);

        // 读取速度指令超时时间
        controller_nh.param(
            "cmd_vel_timeout",
            cmd_vel_timeout_,
            0.5
        );
        // 获取速度指令话题名称
        std::string subscriber_topic;
        controller_nh.param<std::string>(
            "subscriber_topic",
            subscriber_topic,
            "/cmd_vel"
        );

        // 创建订阅器
        cmd_vel_subscriber_ =
            controller_nh.subscribe<geometry_msgs::Twist>(
                subscriber_topic,
                1,
                &HeroChassisController::cmdVelCallback,
                this
            );

        ROS_INFO("Hero chassis controller initialized!");

        return true;
    }
    // 接收底盘速度指令
    void HeroChassisController::cmdVelCallback(
    const geometry_msgs::TwistConstPtr& msg)
    {
        geometry_msgs::TwistStamped command;

        command.header.stamp = ros::Time::now();
        command.twist = *msg;

        command_buffer_.writeFromNonRT(command);
    }
    // 控制器启动
    void HeroChassisController::starting(const ros::Time& time)
    {
        // 记录控制器启动时间
        controller_start_time_ = time;

        // 重置四个 PID
        pid_front_left_.reset();
        pid_front_right_.reset();
        pid_back_left_.reset();
        pid_back_right_.reset();

        // 将四个车轮的初始力矩设为 0
        front_left_joint_.setCommand(0.0);
        front_right_joint_.setCommand(0.0);
        back_left_joint_.setCommand(0.0);
        back_right_joint_.setCommand(0.0);

        ROS_INFO("Hero chassis controller started!");
    }
    // 控制器实时更新
void HeroChassisController::update(
    const ros::Time& time,
    const ros::Duration& period) {
        // 1. 检查控制周期
        if (period.toSec() <= 0.0) {
            front_left_joint_.setCommand(0.0);
            front_right_joint_.setCommand(0.0);
            back_left_joint_.setCommand(0.0);
            back_right_joint_.setCommand(0.0);
            return;
        }

        // 2. 读取速度指令
        const geometry_msgs::TwistStamped* command =
            command_buffer_.readFromRT();

        // 默认底盘速度为零
        Eigen::Vector3d chassis_velocity = Eigen::Vector3d::Zero();

        // 3. 检查速度指令是否有效
        if (command != nullptr) {
            const ros::Time stamp = command->header.stamp;
            const ros::Duration age = time - stamp;

            bool command_valid =
                !stamp.isZero() &&
                stamp >= controller_start_time_ &&
                age.toSec() >= 0.0 &&
                age.toSec() <= cmd_vel_timeout_;

            if (command_valid) {
                chassis_velocity[0] = command->twist.linear.x;
                chassis_velocity[1] = command->twist.linear.y;
                chassis_velocity[2] = command->twist.angular.z;
            }
        }

        // 4. 逆运动学：计算四个车轮的目标角速度
        Eigen::Vector4d target_wheel_velocity =
            algorithm_.inverseKinematics(chassis_velocity);

        // 5. 计算四个车轮的速度误差
        double error_fl =
            target_wheel_velocity[0] - front_left_joint_.getVelocity();

        double error_fr =
            target_wheel_velocity[1] - front_right_joint_.getVelocity();

        double error_bl =
            target_wheel_velocity[2] - back_left_joint_.getVelocity();

        double error_br =
            target_wheel_velocity[3] - back_right_joint_.getVelocity();

        // 6. 四个 PID 分别计算输出力矩
        double effort_fl =
            pid_front_left_.computeCommand(error_fl, period);

        double effort_fr =
            pid_front_right_.computeCommand(error_fr, period);

        double effort_bl =
            pid_back_left_.computeCommand(error_bl, period);

        double effort_br =
            pid_back_right_.computeCommand(error_br, period);

        // 7. 限制输出力矩
        const double max_effort = 5.0;

        auto limit_effort = [max_effort](double effort) {
            return std::max(-max_effort,
                            std::min(max_effort, effort));
        };

        // 8. 将力矩发送给四个车轮
        front_left_joint_.setCommand(limit_effort(effort_fl));
        front_right_joint_.setCommand(limit_effort(effort_fr));
        back_left_joint_.setCommand(limit_effort(effort_bl));
        back_right_joint_.setCommand(limit_effort(effort_br));
    }
        // 控制器停止
        void HeroChassisController::stopping(const ros::Time&)
        {
            // 四个车轮停止输出力矩
            front_left_joint_.setCommand(0.0);
            front_right_joint_.setCommand(0.0);
            back_left_joint_.setCommand(0.0);
            back_right_joint_.setCommand(0.0);

            // 重置 PID
            pid_front_left_.reset();
            pid_front_right_.reset();
            pid_back_left_.reset();
            pid_back_right_.reset();

            ROS_INFO("Hero chassis controller stopped!");
        }
}

