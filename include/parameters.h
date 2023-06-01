/*
 * File: parameters.h
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */

#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations
#include <vector>

struct DroneParameters {
    Eigen::Matrix3d inertiaMatrix;
};

namespace Parameters {
    extern std::vector<DroneParameters> droneParams;
    void loadParameters();
}
