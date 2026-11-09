#include "hero_chassis_controller/Algorithm.hpp"

#include <utility>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/count.hpp>

namespace hero_chassis_controller {

using namespace boost::accumulators;

struct Algorithm::Data {
  accumulator_set<double, features<tag::mean, tag::count>> acc;
};

Algorithm::Algorithm() {
  data_ = std::make_unique<Data>();
}

Algorithm::~Algorithm() = default;

void Algorithm::addData(const double data) {
  data_->acc(data);
}

void Algorithm::addData(const Eigen::VectorXd &data) {
  for (auto i = 0; i < data.size(); ++i)
    addData(data[i]);
}

double Algorithm::getAverage() const {
  return count(data_->acc) ? mean(data_->acc) : 0;
}

Eigen::Vector4d Algorithm::inverseKinematics(
    const Eigen::Vector3d &chassis_velocity) const {

  double vx = chassis_velocity[0];
  double vy = chassis_velocity[1];
  double wz = chassis_velocity[2];

  double R = 0.07625;
  double L = 0.4;

  Eigen::Vector4d wheel_velocity;

  wheel_velocity[0] = (vx + vy + L * wz) / R;
  wheel_velocity[1] = (vx - vy - L * wz) / R;
  wheel_velocity[2] = (vx + vy - L * wz) / R;
  wheel_velocity[3] = (vx - vy + L * wz) / R;

  return wheel_velocity;
}
Eigen::Vector3d Algorithm::forwardKinematics(
    const Eigen::Vector4d &wheel_velocity) const {

  double R = 0.07625;
  double L = 0.4;

  double w1 = wheel_velocity[0];
  double w2 = wheel_velocity[1];
  double w3 = wheel_velocity[2];
  double w4 = wheel_velocity[3];

  Eigen::Vector3d chassis_velocity;

  chassis_velocity[0] = R / 4 * (w1 + w2 + w3 + w4);

  chassis_velocity[1] = R / 4 * (w1 - w2 + w3 - w4);

  chassis_velocity[2] = R / (4 * L) * (w1 - w2 - w3 + w4);

  return chassis_velocity;
}
} /* namespace */
