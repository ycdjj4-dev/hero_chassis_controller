#pragma once

#include <memory>

#include <Eigen/Core>

namespace hero_chassis_controller {

/*!
 * Class containing the algorithmic part of the package.
 */
class Algorithm {
 public:
  /*!
   * Constructor.
   */
  Algorithm();

  /*!
   * Destructor.
   */
  virtual ~Algorithm();

  /*!
   * Add new measurement data.
   * @param data the new data.
   */
  void addData(const double data);

  /*!
   * Add multiple measurements as once.
   * @param data new data.
   */
  void addData(const Eigen::VectorXd &data);

  /*!
   * Get the computed average of the data.
   * @return the average of the data.
   */
  double getAverage() const;
 /*!
 * Calculate wheel velocities.
 */
 Eigen::Vector4d inverseKinematics(
     const Eigen::Vector3d &chassis_velocity) const;
 Eigen::Vector3d forwardKinematics(
     const Eigen::Vector4d &wheel_velocity) const;

 private:

  //! Forward declared structure that will contain the data
  struct Data;

  //! Pointer to data (pimpl)
  std::unique_ptr<Data> data_;
};

} /* namespace */
