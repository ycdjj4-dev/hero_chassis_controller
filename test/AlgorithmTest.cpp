#include "hero_chassis_controller/Algorithm.hpp"

// gtest
#include <gtest/gtest.h>

// STD
#include <vector>

#include <Eigen/Core>

using namespace hero_chassis_controller;

TEST(Algorithm, getWithoutSet
) {
Algorithm algorithm;
const double average = algorithm.getAverage();
EXPECT_EQ(0.0, average);
}

TEST(Algorithm, singleDataPoint
) {
const double inputData = 100.0 * (double) rand() / RAND_MAX;
Algorithm algorithm;
algorithm.
addData(inputData);
const double average = algorithm.getAverage();
EXPECT_NEAR(inputData, average,
1e-10);
}

TEST(Algorithm, singleDataVector
) {
const double inputValue = 100.0 * (double) rand() / RAND_MAX;
Algorithm algorithm;
Eigen::VectorXd inputData;
inputData.resize(2);
inputData << inputValue, 3 *
inputValue;
algorithm.
addData(inputData);
const double average = algorithm.getAverage();
EXPECT_NEAR(2 * inputValue, average, 1e-10);
}

TEST(Algorithm, multipleDataPoints
) {
size_t nMeasurements = 100;
std::vector<double> inputData(nMeasurements);
double sum = 0.0;
for (
auto &data
: inputData) {
data = 100.0 * (double) rand() / RAND_MAX;
sum +=
data;
}

Algorithm algorithm;
for (
const auto data
: inputData) {
algorithm.
addData(data);
}
const double average = algorithm.getAverage();
EXPECT_NEAR(sum
/ nMeasurements, average, 1e-10);
}
TEST(Algorithm, inverseKinematicsForward) {
    Algorithm algorithm;
    Eigen::Vector3d velocity(1.0, 0.0, 0.0);
    Eigen::Vector4d result = algorithm.inverseKinematics(velocity);

    double expected = 1.0 / 0.07625;

    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(expected, result[i], 1e-10);
    }
}

TEST(Algorithm, inverseKinematicsLateral) {
    Algorithm algorithm;
    Eigen::Vector3d velocity(0.0, 1.0, 0.0);
    Eigen::Vector4d result = algorithm.inverseKinematics(velocity);

    double expected[4] = {
        1.0 / 0.07625,
        -1.0 / 0.07625,
        1.0 / 0.07625,
        -1.0 / 0.07625
    };

    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-10);
    }
}

TEST(Algorithm, inverseKinematicsRotation) {
    Algorithm algorithm;
    Eigen::Vector3d velocity(0.0, 0.0, 1.0);
    Eigen::Vector4d result = algorithm.inverseKinematics(velocity);

    double L = 0.4;
    double R = 0.07625;

    double expected[4] = {
        L / R,
        -L / R,
        -L / R,
        L / R
    };

    for (int i = 0; i < 4; ++i) {
        EXPECT_NEAR(expected[i], result[i], 1e-10);
    }
}
TEST(Algorithm, forwardAndInverseKinematics) {
    Algorithm algorithm;

    Eigen::Vector3d input;
    input << 1.0, 0.3, 0.2;

    Eigen::Vector4d wheel_velocity =
        algorithm.inverseKinematics(input);

    Eigen::Vector3d output =
        algorithm.forwardKinematics(wheel_velocity);

    EXPECT_NEAR(input[0], output[0], 1e-10);
    EXPECT_NEAR(input[1], output[1], 1e-10);
    EXPECT_NEAR(input[2], output[2], 1e-10);
}
