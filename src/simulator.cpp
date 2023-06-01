/*
 * File: simulator.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "drone.h"
#include "parameters.h"

int main() {
    Parameters::loadParameters();

    std::vector<Drone> drones;
    for (int i = 0; i < Parameters::droneParams.size(); ++i) {
        drones.emplace_back(i);
    }

    double timeStep = 0.01;
    int numSteps = 1000;

    for (int step = 0; step < numSteps; ++step) {
        for (auto& drone : drones) {
            drone.updateState(timeStep);
        }

        // Perform other simulation tasks

        for (auto& drone : drones) {
            drone.setExternalTorque(Eigen::Vector3d(0.0, 0.0, 0.0));
        }

        // Perform other simulation tasks
    }

    return 0;
}
