// 运动学算法实现。
#include "hero_chassis_controller/Algorithm.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/mean.hpp>

namespace hero_chassis_controller {
    struct Algorithm::Data {
        using Accumulator = boost::accumulators::accumulator_set<
            double,
            boost::accumulators::features<boost::accumulators::tag::mean,
                boost::accumulators::tag::count> >;
        Accumulator acc;
    };

    Algorithm::Algorithm() : data_(std::make_unique<Data>()) {
    }

    Algorithm::~Algorithm() = default;

    void Algorithm::addData(const double data) { data_->acc(data); }

    void Algorithm::addData(const Eigen::VectorXd &data) {
        for (Eigen::Index i = 0; i < data.size(); ++i) {
            addData(data[i]);
        }
    }

    double Algorithm::getAverage() const {
        if (boost::accumulators::count(data_->acc) == 0) {
            return 0.0;
        }
        return boost::accumulators::mean(data_->acc);
    }

    Eigen::Vector4d Algorithm::inverseKinematics(
        const Eigen::Vector3d &chassis_velocity) const {
        const double vx = chassis_velocity[0];
        const double vy = chassis_velocity[1];
        const double wz = chassis_velocity[2];

        // 车轮半径 (m)。
        const double kWheelRadius = 0.07625;
        // 运动学系数 L = a + b，a 为半轴距 (wheel_base / 2)，
        // b 为半轮距 (wheel_track / 2)。
        // 本模型 wheel_base = wheel_track = 0.4，故 L = 0.2 + 0.2 = 0.4。
        const double kHalfWheelbasePlusHalfTrack = 0.4;

        // 经典麦轮逆运动学，与 rm_description_for_task 模型的辊子布局一致
        // (reflect: 左前=1, 右前=-1, 左后=-1, 右后=1；四轮关节轴均朝 +y)。
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
