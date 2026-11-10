// 麦轮底盘控制器实现。
#include "hero_chassis_controller/HeroChassisController.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace hero_chassis_controller {
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *hw,
                                     ros::NodeHandle & /*root_nh*/,
                                     ros::NodeHandle &controller_nh) {
        try {
            front_left_joint_ = hw->getHandle("left_front_wheel_joint");
            front_right_joint_ = hw->getHandle("right_front_wheel_joint");
            back_left_joint_ = hw->getHandle("left_back_wheel_joint");
            back_right_joint_ = hw->getHandle("right_back_wheel_joint");

            // 创建四个 PID 对应的参数命名空间。
            ros::NodeHandle pid_front_left_nh(controller_nh, "pid_front_left");
            ros::NodeHandle pid_front_right_nh(controller_nh, "pid_front_right");
            ros::NodeHandle pid_back_left_nh(controller_nh, "pid_back_left");
            ros::NodeHandle pid_back_right_nh(controller_nh, "pid_back_right");

            // 初始化四个车轮的 PID。
            if (!pid_front_left_.init(pid_front_left_nh)) {
                ROS_ERROR("Failed to initialize front left PID");
                return false;
            }
            if (!pid_front_right_.init(pid_front_right_nh)) {
                ROS_ERROR("Failed to initialize front right PID");
                return false;
            }
            if (!pid_back_left_.init(pid_back_left_nh)) {
                ROS_ERROR("Failed to initialize back left PID");
                return false;
            }
            if (!pid_back_right_.init(pid_back_right_nh)) {
                ROS_ERROR("Failed to initialize back right PID");
                return false;
            }
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM("Failed to get joint handles: " << e.what());
            return false;
        }

        geometry_msgs::TwistStamped initial_command;
        command_buffer_.initRT(initial_command);

        // 读取速度指令超时时间。
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
        // 读取摩擦前馈增益。
        controller_nh.param("feedforward_gain", feedforward_gain_, 0.05);

        // 读取速度指令话题名称。
        std::string subscriber_topic;
        controller_nh.param<std::string>("subscriber_topic", subscriber_topic,
                                         "/cmd_vel");

        // 创建订阅器。
        cmd_vel_subscriber_ = controller_nh.subscribe<geometry_msgs::Twist>(
            subscriber_topic, 1, &HeroChassisController::cmdVelCallback, this);

        // 初始化里程计发布器。
        odom_publisher_ = std::make_unique<
            realtime_tools::RealtimePublisher<nav_msgs::Odometry> >(controller_nh,
                                                                    "odom", 10);
        // 初始化 TF 发布器。
        tf_publisher_ = std::make_unique<
            realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> >(controller_nh,
                                                                     "/tf", 10);

        // 预先分配一个 TF 消息。
        tf_publisher_->msg_.transforms.resize(1);

        // 设置里程计坐标系。
        odom_publisher_->msg_.header.frame_id = "odom";
        odom_publisher_->msg_.child_frame_id = "base_link";

        // 设置 TF 坐标系。
        tf_publisher_->msg_.transforms[0].header.frame_id = "odom";
        tf_publisher_->msg_.transforms[0].child_frame_id = "base_link";

        // 初始化四元数，表示初始朝向为 0。
        odom_publisher_->msg_.pose.pose.orientation.w = 1.0;
        tf_publisher_->msg_.transforms[0].transform.rotation.w = 1.0;

        ROS_INFO("Hero chassis controller initialized!");
        return true;
    }

    // 接收底盘速度指令。
    void HeroChassisController::cmdVelCallback(
        const geometry_msgs::TwistConstPtr &msg) {
        geometry_msgs::TwistStamped command;
        command.header.stamp = ros::Time::now();
        command.twist = *msg;
        command_buffer_.writeFromNonRT(command);
    }

    // 控制器启动。
    void HeroChassisController::starting(const ros::Time &time) {
        // 记录控制器启动时间。
        controller_start_time_ = time;

        // 重置里程计累计位姿。
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_yaw_ = 0.0;

        // 重置四个 PID。
        pid_front_left_.reset();
        pid_front_right_.reset();
        pid_back_left_.reset();
        pid_back_right_.reset();

        // 将四个车轮的初始力矩设为 0。
        front_left_joint_.setCommand(0.0);
        front_right_joint_.setCommand(0.0);
        back_left_joint_.setCommand(0.0);
        back_right_joint_.setCommand(0.0);

        ROS_INFO("Hero chassis controller started!");
    }

    // 控制器实时更新。
    void HeroChassisController::update(const ros::Time &time,
                                       const ros::Duration &period) {
        // 1. 检查控制周期。
        if (period.toSec() <= 0.0) {
            front_left_joint_.setCommand(0.0);
            front_right_joint_.setCommand(0.0);
            back_left_joint_.setCommand(0.0);
            back_right_joint_.setCommand(0.0);
            return;
        }

        // 2. 读取速度指令。
        const geometry_msgs::TwistStamped *command = command_buffer_.readFromRT();

        // 默认底盘速度为零。
        Eigen::Vector3d chassis_velocity = Eigen::Vector3d::Zero();

        // 3. 检查速度指令是否有效。
        if (command != nullptr) {
            const ros::Time stamp = command->header.stamp;
            const ros::Duration age = time - stamp;

            const bool command_valid = !stamp.isZero() &&
                                       stamp >= controller_start_time_ &&
                                       age.toSec() >= 0.0 &&
                                       age.toSec() <= cmd_vel_timeout_;

            if (command_valid) {
                chassis_velocity[0] = command->twist.linear.x;
                chassis_velocity[1] = command->twist.linear.y;
                chassis_velocity[2] = command->twist.angular.z;
            }
        }

        // 4. 逆运动学：计算四个车轮的目标角速度。
        const Eigen::Vector4d target_wheel_velocity =
                algorithm_.inverseKinematics(chassis_velocity);

        // 5. 计算四个车轮的速度误差。
        const double error_fl =
                target_wheel_velocity[0] - front_left_joint_.getVelocity();
        const double error_fr =
                target_wheel_velocity[1] - front_right_joint_.getVelocity();
        const double error_bl =
                target_wheel_velocity[2] - back_left_joint_.getVelocity();
        const double error_br =
                target_wheel_velocity[3] - back_right_joint_.getVelocity();

        // 6. 四个 PID 分别计算输出力矩，并叠加摩擦前馈。
        const double effort_fl = pid_front_left_.computeCommand(error_fl, period) +
                                 feedforward_gain_ * target_wheel_velocity[0];
        const double effort_fr = pid_front_right_.computeCommand(error_fr, period) +
                                 feedforward_gain_ * target_wheel_velocity[1];
        const double effort_bl = pid_back_left_.computeCommand(error_bl, period) +
                                 feedforward_gain_ * target_wheel_velocity[2];
        const double effort_br = pid_back_right_.computeCommand(error_br, period) +
                                 feedforward_gain_ * target_wheel_velocity[3];

        // 7. 限制输出力矩。
        constexpr double kMaxEffort = 5.0;
        const auto limit_effort = [kMaxEffort](const double effort) {
            return std::max(-kMaxEffort, std::min(kMaxEffort, effort));
        };

        // 8. 将力矩发送给四个车轮。
        front_left_joint_.setCommand(limit_effort(effort_fl));
        front_right_joint_.setCommand(limit_effort(effort_fr));
        back_left_joint_.setCommand(limit_effort(effort_bl));
        back_right_joint_.setCommand(limit_effort(effort_br));

        // 9. 获取四个车轮的实际角速度
        Eigen::Vector4d actual_wheel_velocity;

        actual_wheel_velocity <<
                front_left_joint_.getVelocity(),
                front_right_joint_.getVelocity(),
                back_left_joint_.getVelocity(),
                back_right_joint_.getVelocity();

        // 10. 正运动学：计算底盘实际速度
        const Eigen::Vector3d actual_chassis_velocity =
                algorithm_.forwardKinematics(actual_wheel_velocity);

        // 11. 根据实际速度积分，计算机器人位姿
        const double dt = period.toSec();

        // 使用中间时刻的航向角进行位置积分
        const double yaw_mid =
                odom_yaw_ + 0.5 * actual_chassis_velocity[2] * dt;

        odom_x_ += (
            actual_chassis_velocity[0] * std::cos(yaw_mid) -
            actual_chassis_velocity[1] * std::sin(yaw_mid)
        ) * dt;

        odom_y_ += (
            actual_chassis_velocity[0] * std::sin(yaw_mid) +
            actual_chassis_velocity[1] * std::cos(yaw_mid)
        ) * dt;

        odom_yaw_ += actual_chassis_velocity[2] * dt;

        // 12. 将航向角转换为四元数
        geometry_msgs::Quaternion orientation;

        orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.w = std::cos(odom_yaw_ / 2.0);
        orientation.z = std::sin(odom_yaw_ / 2.0);

        // 13. 发布里程计
        if (odom_publisher_->trylock()) {
            odom_publisher_->msg_.header.stamp = time;

            odom_publisher_->msg_.pose.pose.position.x = odom_x_;
            odom_publisher_->msg_.pose.pose.position.y = odom_y_;

            odom_publisher_->msg_.pose.pose.orientation = orientation;

            odom_publisher_->msg_.twist.twist.linear.x =
                    actual_chassis_velocity[0];

            odom_publisher_->msg_.twist.twist.linear.y =
                    actual_chassis_velocity[1];

            odom_publisher_->msg_.twist.twist.angular.z =
                    actual_chassis_velocity[2];

            odom_publisher_->unlockAndPublish();
        }

        // 14. 发布 TF
        if (tf_publisher_->trylock()) {
            tf_publisher_->msg_.transforms[0].header.stamp = time;

            tf_publisher_->msg_.transforms[0].transform.translation.x =
                    odom_x_;

            tf_publisher_->msg_.transforms[0].transform.translation.y =
                    odom_y_;

            tf_publisher_->msg_.transforms[0].transform.translation.z = 0.0;

            tf_publisher_->msg_.transforms[0].transform.rotation =
                    orientation;

            tf_publisher_->unlockAndPublish();
        }
    }

    // 控制器停止。
    void HeroChassisController::stopping(const ros::Time & /*time*/) {
        // 四个车轮停止输出力矩。
        front_left_joint_.setCommand(0.0);
        front_right_joint_.setCommand(0.0);
        back_left_joint_.setCommand(0.0);
        back_right_joint_.setCommand(0.0);

        // 重置 PID。
        pid_front_left_.reset();
        pid_front_right_.reset();
        pid_back_left_.reset();
        pid_back_right_.reset();

        ROS_INFO("Hero chassis controller stopped!");
    }
} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
