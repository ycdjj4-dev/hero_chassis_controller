// 运动学算法实现: 数据统计 + 麦轮逆/正运动学。
#include "hero_chassis_controller/Algorithm.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>

namespace hero_chassis_controller {
    // 累加器: 用于 addData()/getAverage() 的均值与计数统计。
    struct Algorithm::Data {
        using Accumulator = boost::accumulators::accumulator_set<
            double,
            boost::accumulators::features<boost::accumulators::tag::mean,
                boost::accumulators::tag::count> >;
        Accumulator acc;
    };

    // ---- 板块 1: 数据统计 ----
    Algorithm::Algorithm() : data_(std::make_unique<Data>()) {
    }

    Algorithm::~Algorithm() = default;

    // 添加单个数据。
    void Algorithm::addData(const double data) { data_->acc(data); }

    // 添加一组数据。
    void Algorithm::addData(const Eigen::VectorXd &data) {
        for (Eigen::Index i = 0; i < data.size(); ++i) {
            addData(data[i]);
        }
    }

    // 返回平均值; 无数据时返回 0。
    double Algorithm::getAverage() const {
        if (boost::accumulators::count(data_->acc) == 0) {
            return 0.0;
        }
        return boost::accumulators::mean(data_->acc);
    }

    // ---- 板块 2: 逆运动学(选做5) ----
    // 输入: 底盘速度(vx, vy, wz) -> 输出: 四轮目标转速 [左前,右前,左后,右后]
    Eigen::Vector4d Algorithm::inverseKinematics(
        const Eigen::Vector3d &chassis_velocity) const {
        const double vx = chassis_velocity[0];
        const double vy = chassis_velocity[1];
        const double wz = chassis_velocity[2];

        // 车轮半径 (m)。
        const double kWheelRadius = 0.07625;
        // 运动学系数 L = a + b (半轴距 + 半轮距), 本模型为 0.2 + 0.2 = 0.4。
        const double kHalfWheelbasePlusHalfTrack = 0.4;

        // 经典麦轮公式, 符号与模型辊子布局一致(reflect: 左前/右后=1, 右前/左后=-1)。
        Eigen::Vector4d wheel_velocity;
        wheel_velocity[0] =
                (vx - vy - kHalfWheelbasePlusHalfTrack * wz) / kWheelRadius; // 左前
        wheel_velocity[1] =
                (vx + vy + kHalfWheelbasePlusHalfTrack * wz) / kWheelRadius; // 右前
        wheel_velocity[2] =
                (vx + vy - kHalfWheelbasePlusHalfTrack * wz) / kWheelRadius; // 左后
        wheel_velocity[3] =
                (vx - vy + kHalfWheelbasePlusHalfTrack * wz) / kWheelRadius; // 右后

        return wheel_velocity;
    }

    // ---- 板块 3: 正运动学(选做6 里程计的基础) ----
    // 输入: 四轮实际转速 -> 输出: 底盘实际速度(vx, vy, wz)
    Eigen::Vector3d Algorithm::forwardKinematics(
        const Eigen::Vector4d &wheel_velocity) const {
        const double kWheelRadius = 0.07625;
        const double kHalfWheelbasePlusHalfTrack = 0.4;

        const double w_fl = wheel_velocity[0];
        const double w_fr = wheel_velocity[1];
        const double w_bl = wheel_velocity[2];
        const double w_br = wheel_velocity[3];

        Eigen::Vector3d chassis_velocity;
        chassis_velocity[0] = kWheelRadius / 4 * (w_fl + w_fr + w_bl + w_br);
        chassis_velocity[1] = kWheelRadius / 4 * (-w_fl + w_fr + w_bl - w_br);
        chassis_velocity[2] = kWheelRadius / (4 * kHalfWheelbasePlusHalfTrack) *
                              (-w_fl + w_fr - w_bl + w_br);

        return chassis_velocity;
    }
} // namespace hero_chassis_controller
