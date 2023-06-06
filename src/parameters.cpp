/*
 * File: parameters.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "parameters.h"

namespace Parameters {
    std::vector<DroneParameters> droneParams;

    void loadParameters() {
        // Load the drone parameters from a file or any other source
        // Here, we manually define the parameters for 4 drones as an example

        // Drone 1 parameters
        DroneParameters drone1Params;
        drone1Params.droneType = DroneTypes::MC_QUAD;
        drone1Params.mass = 1.0; // Set the mass for drone 1
        drone1Params.inertiaMatrix << 1.0, 0.0, 0.0,
                                      0.0, 2.0, 0.0,
                                      0.0, 0.0, 3.0;
        drone1Params.initPos << 0.0, 0.0, 0.0;
        drone1Params.altCtrlPID.Kp = 1.0;
        drone1Params.altCtrlPID.Ki = 0.01;
        drone1Params.altCtrlPID.Kd = 0.1;
        droneParams.push_back(drone1Params);

        // Drone 2 parameters
        DroneParameters drone2Params;
        drone2Params.droneType = DroneTypes::MC_HEXA;
        drone2Params.mass = 1.5; // Set the mass for drone 2
        drone2Params.inertiaMatrix << 4.0, 0.0, 0.0,
                                      0.0, 5.0, 0.0,
                                      0.0, 0.0, 6.0;
        drone2Params.initPos << 0.5, 0.0, 0.0;
        drone2Params.altCtrlPID.Kp = 1.0;
        drone2Params.altCtrlPID.Ki = 0.0;
        drone2Params.altCtrlPID.Kd = 0.2;
        droneParams.push_back(drone2Params);

        // Drone 3 parameters
        DroneParameters drone3Params;
        drone3Params.droneType = DroneTypes::MC_OCTO;
        drone3Params.mass = 2.0; // Set the mass for drone 3
        drone3Params.inertiaMatrix << 7.0, 0.0, 0.0,
                                      0.0, 8.0, 0.0,
                                      0.0, 0.0, 9.0;
        drone3Params.initPos <<1.0, 0.0, 0.0;
        drone3Params.altCtrlPID.Kp = 1.0;
        drone3Params.altCtrlPID.Ki = 0.0;
        drone3Params.altCtrlPID.Kd = 0.2;
        droneParams.push_back(drone3Params);
    }
}