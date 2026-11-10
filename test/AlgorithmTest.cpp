// Algorithm 类的单元测试。
#include "hero_chassis_controller/Algorithm.hpp"

#include <cstdlib>
#include <vector>

#include <Eigen/Core>

#include <gtest/gtest.h>

namespace {
    using hero_chassis_controller::Algorithm;

    TEST(Algorithm, GetWithoutSet) {
        Algorithm algorithm;
        const double average = algorithm.getAverage();
        EXPECT_EQ(0.0, average);
    }

    TEST(Algorithm, SingleDataPoint) {
        const double input_data = 100.0 * std::rand() / RAND_MAX;
        Algorithm algorithm;
        algorithm.addData(input_data);
        const double average = algorithm.getAverage();
        EXPECT_NEAR(input_data, average, 1e-10);
    }

    TEST(Algorithm, SingleDataVector) {
        const double input_value = 100.0 * std::rand() / RAND_MAX;
        Algorithm algorithm;
        Eigen::VectorXd input_data;
        input_data.resize(2);
        input_data << input_value, 3 * input_value;
        algorithm.addData(input_data);
        const double average = algorithm.getAverage();
        EXPECT_NEAR(2 * input_value, average, 1e-10);
    }

    TEST(Algorithm, MultipleDataPoints) {
        const size_t kNumMeasurements = 100;
        std::vector<double> input_data(kNumMeasurements);
        double sum = 0.0;
        for (double &data: input_data) {
            data = 100.0 * std::rand() / RAND_MAX;
            sum += data;
        }

        Algorithm algorithm;
        for (const double data: input_data) {
            algorithm.addData(data);
        }
        const double average = algorithm.getAverage();
        EXPECT_NEAR(sum / kNumMeasurements, average, 1e-10);
    }

    TEST(Algorithm, InverseKinematicsForward) {
        Algorithm algorithm;
        const Eigen::Vector3d velocity(1.0, 0.0, 0.0);
        const Eigen::Vector4d result = algorithm.inverseKinematics(velocity);

        const double expected = 1.0 / 0.07625;
        for (int i = 0; i < 4; ++i) {
            EXPECT_NEAR(expected, result[i], 1e-10);
        }
    }

    TEST(Algorithm, InverseKinematicsLateral) {
        Algorithm algorithm;
        const Eigen::Vector3d velocity(0.0, 1.0, 0.0);
        const Eigen::Vector4d result = algorithm.inverseKinematics(velocity);

        const double expected[4] = {
            -1.0 / 0.07625, 1.0 / 0.07625,
            1.0 / 0.07625, -1.0 / 0.07625
        };
        for (int i = 0; i < 4; ++i) {
            EXPECT_NEAR(expected[i], result[i], 1e-10);
        }
    }

    TEST(Algorithm, InverseKinematicsRotation) {
        Algorithm algorithm;
        const Eigen::Vector3d velocity(0.0, 0.0, 1.0);
        const Eigen::Vector4d result = algorithm.inverseKinematics(velocity);

        const double kL = 0.4;
        const double kR = 0.07625;
        const double expected[4] = {-kL / kR, kL / kR, -kL / kR, kL / kR};
        for (int i = 0; i < 4; ++i) {
            EXPECT_NEAR(expected[i], result[i], 1e-10);
        }
    }

    TEST(Algorithm, ForwardAndInverseKinematics) {
        Algorithm algorithm;

        Eigen::Vector3d input;
        input << 1.0, 0.3, 0.2;

        const Eigen::Vector4d wheel_velocity = algorithm.inverseKinematics(input);
        const Eigen::Vector3d output = algorithm.forwardKinematics(wheel_velocity);

        EXPECT_NEAR(input[0], output[0], 1e-10);
        EXPECT_NEAR(input[1], output[1], 1e-10);
        EXPECT_NEAR(input[2], output[2], 1e-10);
    }
} // namespace
