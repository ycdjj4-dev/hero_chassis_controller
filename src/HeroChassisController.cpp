// 麦轮底盘控制器实现。
// init: 加载时初始化 | cmdVelCallback: 订阅回调 | starting/stopping: 启停清理
// update: 每周期执行一次 —— 指令处理 -> 运动控制(PID) -> 里程计发布
#include "hero_chassis_controller/HeroChassisController.hpp"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace hero_chassis_controller {
    // init(): 加载时执行一次, 连接硬件句柄、读取参数、创建收发器。
    bool HeroChassisController::init(hardware_interface::EffortJointInterface *hw,
                                     ros::NodeHandle & /*root_nh*/,
                                     ros::NodeHandle &controller_nh) {
        // ---- 板块 1: 关节句柄与 PID 初始化 ----
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

        // ---- 板块 2: 读取配置参数 ----
        geometry_msgs::TwistStamped initial_command;
        command_buffer_.initRT(initial_command);

        // 指令超时时间。
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
        // 摩擦前馈增益。
        controller_nh.param("feedforward_gain", feedforward_gain_, 0.05);
        // 速度指令坐标系模式(选做7)。
        controller_nh.param<std::string>("velocity_mode", velocity_mode_,
                                         "chassis");
        if (velocity_mode_ != "chassis" && velocity_mode_ != "world") {
            ROS_WARN_STREAM("Unknown velocity_mode '" << velocity_mode_
                << "', fallback to 'chassis'");
            velocity_mode_ = "chassis";
        }
        // 速度指令话题名。
        std::string subscriber_topic;
        controller_nh.param<std::string>("subscriber_topic", subscriber_topic,
                                         "/cmd_vel");

        // ---- 板块 3: 创建订阅器与发布器 ----
        // 订阅 /cmd_vel。
        cmd_vel_subscriber_ = controller_nh.subscribe<geometry_msgs::Twist>(
            subscriber_topic, 1, &HeroChassisController::cmdVelCallback, this);

        // 发布 /odom(选做6)。
        odom_publisher_ = std::make_unique<
            realtime_tools::RealtimePublisher<nav_msgs::Odometry> >(controller_nh,
                                                                    "/odom", 10);
        // 发布 /tf(选做6)。
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

        // 初始朝向为 0。
        odom_publisher_->msg_.pose.pose.orientation.w = 1.0;
        tf_publisher_->msg_.transforms[0].transform.rotation.w = 1.0;

        ROS_INFO("Hero chassis controller initialized!");
        return true;
    }

    // 订阅回调: 把收到的指令(带时间戳)写入实时缓冲区。
    void HeroChassisController::cmdVelCallback(
        const geometry_msgs::TwistConstPtr &msg) {
        geometry_msgs::TwistStamped command;
        command.header.stamp = ros::Time::now();
        command.twist = *msg;
        command_buffer_.writeFromNonRT(command);
    }

    // 启动清理: 清零力矩、里程计位姿与 PID 状态。
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

        // 初始力矩清零。
        front_left_joint_.setCommand(0.0);
        front_right_joint_.setCommand(0.0);
        back_left_joint_.setCommand(0.0);
        back_right_joint_.setCommand(0.0);

        ROS_INFO("Hero chassis controller started!");
    }

    // update(): 每周期一次的控制流水线。
    //   步骤 1~3  指令处理, 含 world 模式坐标旋转(选做7)
    //   步骤 4~8  逆运动学 + 四轮 PID 力矩控制(选做4/5)
    //   步骤 9~14 正运动学积分, 发布 /odom 与 TF(选做6)
    void HeroChassisController::update(const ros::Time &time,
                                       const ros::Duration &period) {
        // ---- 板块 A: 指令处理(步骤 1~3) ----
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

        // 3. 检查指令是否有效(时间戳、超时)。
        if (command != nullptr) {
            const ros::Time stamp = command->header.stamp;
            const ros::Duration age = time - stamp;

            const bool command_valid = !stamp.isZero() &&
                                       stamp >= controller_start_time_ &&
                                       age.toSec() >= 0.0 &&
                                       age.toSec() <= cmd_vel_timeout_;

            if (command_valid) {
                // world 模式: 用里程计航向角把世界系速度旋转到底盘系(选做7)。
                double vx = command->twist.linear.x;
                double vy = command->twist.linear.y;
                if (velocity_mode_ == "world") {
                    const double cos_yaw = std::cos(odom_yaw_);
                    const double sin_yaw = std::sin(odom_yaw_);
                    const double vx_world = vx;
                    const double vy_world = vy;
                    vx = cos_yaw * vx_world + sin_yaw * vy_world;
                    vy = -sin_yaw * vx_world + cos_yaw * vy_world;
                }
                // wz 与坐标系无关，无需变换。
                chassis_velocity[0] = vx;
                chassis_velocity[1] = vy;
                chassis_velocity[2] = command->twist.angular.z;
            }
        }

        // ---- 板块 B: 运动控制(步骤 4~8) ----
        // 4. 逆运动学: 底盘速度 -> 四轮目标转速。
        const Eigen::Vector4d target_wheel_velocity =
                algorithm_.inverseKinematics(chassis_velocity);

        // 5. 速度误差 = 目标转速 - 实际转速。
        const double error_fl =
                target_wheel_velocity[0] - front_left_joint_.getVelocity();
        const double error_fr =
                target_wheel_velocity[1] - front_right_joint_.getVelocity();
        const double error_bl =
                target_wheel_velocity[2] - back_left_joint_.getVelocity();
        const double error_br =
                target_wheel_velocity[3] - back_right_joint_.getVelocity();

        // 6. PID 计算力矩 + 摩擦前馈。
        const double effort_fl = pid_front_left_.computeCommand(error_fl, period) +
                                 feedforward_gain_ * target_wheel_velocity[0];
        const double effort_fr = pid_front_right_.computeCommand(error_fr, period) +
                                 feedforward_gain_ * target_wheel_velocity[1];
        const double effort_bl = pid_back_left_.computeCommand(error_bl, period) +
                                 feedforward_gain_ * target_wheel_velocity[2];
        const double effort_br = pid_back_right_.computeCommand(error_br, period) +
                                 feedforward_gain_ * target_wheel_velocity[3];

        // 7. 力矩限幅。
        constexpr double kMaxEffort = 5.0;
        const auto limit_effort = [kMaxEffort](const double effort) {
            return std::max(-kMaxEffort, std::min(kMaxEffort, effort));
        };

        // 8. 下发力矩。
        front_left_joint_.setCommand(limit_effort(effort_fl));
        front_right_joint_.setCommand(limit_effort(effort_fr));
        back_left_joint_.setCommand(limit_effort(effort_bl));
        back_right_joint_.setCommand(limit_effort(effort_br));

        // ---- 板块 C: 里程计(步骤 9~14) ----
        // 9. 读取四轮实际转速。
        Eigen::Vector4d actual_wheel_velocity;

        actual_wheel_velocity <<
                front_left_joint_.getVelocity(),
                front_right_joint_.getVelocity(),
                back_left_joint_.getVelocity(),
                back_right_joint_.getVelocity();

        // 10. 正运动学: 实际轮速 -> 底盘实际速度。
        const Eigen::Vector3d actual_chassis_velocity =
                algorithm_.forwardKinematics(actual_wheel_velocity);

        // 11. 积分位姿(用中间时刻航向角, 比欧拉积分更准)。
        const double dt = period.toSec();
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

        // 12. 航向角转四元数。
        geometry_msgs::Quaternion orientation;

        orientation.x = 0.0;
        orientation.y = 0.0;
        orientation.w = std::cos(odom_yaw_ / 2.0);
        orientation.z = std::sin(odom_yaw_ / 2.0);

        // 13. 发布 /odom。
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

        // 14. 发布 TF(odom -> base_link)。
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

    // 停止清理: 力矩清零并重置 PID。
    void HeroChassisController::stopping(const ros::Time & /*time*/) {
        // 力矩清零。
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
