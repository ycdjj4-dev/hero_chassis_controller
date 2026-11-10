// 运动学算法接口：麦轮逆/正运动学与数据统计。
#ifndef HERO_CHASSIS_CONTROLLER_ALGORITHM_HPP_
#define HERO_CHASSIS_CONTROLLER_ALGORITHM_HPP_

#include <memory>

#include <Eigen/Core>

namespace hero_chassis_controller {
    // 运动学算法类：提供麦轮逆/正运动学计算与数据统计功能。
    class Algorithm {
    public:
        Algorithm();

        virtual ~Algorithm();

        // 添加单个测量数据。
        void addData(double data);

        // 添加一组测量数据。
        void addData(const Eigen::VectorXd &data);

        // 返回已添加数据的平均值；无数据时返回 0。
        double getAverage() const;

        // 逆运动学：由底盘速度 (vx, vy, wz) 计算四轮目标角速度 (rad/s)。
        // 返回顺序为 [左前, 右前, 左后, 右后]。
        Eigen::Vector4d inverseKinematics(
            const Eigen::Vector3d &chassis_velocity) const;

        // 正运动学：由四轮角速度计算底盘速度 (vx, vy, wz)。
        Eigen::Vector3d forwardKinematics(
            const Eigen::Vector4d &wheel_velocity) const;

    private:
        // 前向声明的实现结构 (pimpl)。
        struct Data;

        // 指向实现数据的指针。
        std::unique_ptr<Data> data_;
    };
} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_ALGORITHM_HPP_
