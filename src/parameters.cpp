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
        /*
        ******************
        Drone 1 parameters
        ******************
        */
        DroneParameters drone1Params;
        // type
        drone1Params.droneType = DroneTypes::MC_QUAD;
        // physical properties
        drone1Params.mass = 1.0; // Set the mass for drone 1
        drone1Params.inertiaMatrix << 1.0, 0.0, 0.0,
                                      0.0, 2.0, 0.0,
                                      0.0, 0.0, 3.0;
        // init states
        drone1Params.initPos << 0.0, 0.0, 0.0;
        // alt pid ctrl
        double omega = 3;
        double xi = 1;
        drone1Params.altCtrlPID.Kp = pow(omega,2);
        drone1Params.altCtrlPID.Ki = 0.1;
        drone1Params.altCtrlPID.Kd = 2*omega*xi;
        // att tilt prio ctrl
        double timeConst_X = 0.3;
        double timeConst_Y = 0.3;
        double timeConst_Z  = 0.3;
        double damping_X = 0.707;
        double damping_Y = 0.707;
        double damping_Z = 0.707;
        drone1Params.attCtrlTiltPrio.KP << 2*drone1Params.inertiaMatrix.coeff(0,0)/(timeConst_X*timeConst_X), 0.0, 0.0,
                                           0.0, 2*drone1Params.inertiaMatrix.coeff(1,1)/(timeConst_Y*timeConst_Y), 0.0,
                                           0.0, 0.0, 2*drone1Params.inertiaMatrix.coeff(2,2)/(timeConst_Z*timeConst_Z);
        drone1Params.attCtrlTiltPrio.KD << 2*drone1Params.inertiaMatrix.coeff(0,0)*damping_X/timeConst_X, 0.0, 0.0,
                                           0.0, 2*drone1Params.inertiaMatrix.coeff(1,1)*damping_Y/timeConst_Y, 0.0,
                                           0.0, 0.0, 2*drone1Params.inertiaMatrix.coeff(2,2)*damping_Z/timeConst_Z;
        // put them all in droneParams
        droneParams.push_back(drone1Params);

        /*
        ******************
        Drone 2 parameters
        ******************
        */
        DroneParameters drone2Params;
        // type
        drone2Params.droneType = DroneTypes::MC_HEXA;
        // physical properties
        drone2Params.mass = 1.5; // Set the mass for drone 2
        drone2Params.inertiaMatrix << 4.0, 0.0, 0.0,
                                      0.0, 5.0, 0.0,
                                      0.0, 0.0, 6.0;
        // init states
        drone2Params.initPos << 0.5, 0.0, 0.0;
        // alt pid ctrl
        double omega = 3;
        double xi = 1;
        drone2Params.altCtrlPID.Kp = pow(omega,2);
        drone2Params.altCtrlPID.Ki = 0.1;
        drone2Params.altCtrlPID.Kd = 2*omega*xi;
        // att tilt prio ctrl
        double timeConst_X = 0.3;
        double timeConst_Y = 0.3;
        double timeConst_Z  = 0.3;
        double damping_X = 0.707;
        double damping_Y = 0.707;
        double damping_Z = 0.707;
        drone2Params.attCtrlTiltPrio.KP << 2*drone2Params.inertiaMatrix.coeff(0,0)/(timeConst_X*timeConst_X), 0.0, 0.0,
                                           0.0, 2*drone2Params.inertiaMatrix.coeff(1,1)/(timeConst_Y*timeConst_Y), 0.0,
                                           0.0, 0.0, 2*drone2Params.inertiaMatrix.coeff(2,2)/(timeConst_Z*timeConst_Z);
        drone2Params.attCtrlTiltPrio.KD << 2*drone2Params.inertiaMatrix.coeff(0,0)*damping_X/timeConst_X, 0.0, 0.0,
                                           0.0, 2*drone2Params.inertiaMatrix.coeff(1,1)*damping_Y/timeConst_Y, 0.0,
                                           0.0, 0.0, 2*drone2Params.inertiaMatrix.coeff(2,2)*damping_Z/timeConst_Z;
        // put them all in droneParams
        droneParams.push_back(drone2Params);

        /*
        ******************
        Drone 3 parameters
        ******************
        */
        DroneParameters drone3Params;
        // type
        drone3Params.droneType = DroneTypes::MC_OCTO;
        // physical properties
        drone3Params.mass = 2.0; // Set the mass for drone 3
        drone3Params.inertiaMatrix << 7.0, 0.0, 0.0,
                                      0.0, 8.0, 0.0,
                                      0.0, 0.0, 9.0;
        // init states
        drone3Params.initPos <<1.0, 0.0, 0.0;
        // alt pid ctrl
        double omega = 3;
        double xi = 1;
        drone3Params.altCtrlPID.Kp = pow(omega,2);
        drone3Params.altCtrlPID.Ki = 0.1;
        drone3Params.altCtrlPID.Kd = 2*omega*xi;
        // att tilt prio ctrl
        double timeConst_X = 0.3;
        double timeConst_Y = 0.3;
        double timeConst_Z  = 0.3;
        double damping_X = 0.707;
        double damping_Y = 0.707;
        double damping_Z = 0.707;
        drone3Params.attCtrlTiltPrio.KP << 2*drone3Params.inertiaMatrix.coeff(0,0)/(timeConst_X*timeConst_X), 0.0, 0.0,
                                           0.0, 2*drone3Params.inertiaMatrix.coeff(1,1)/(timeConst_Y*timeConst_Y), 0.0,
                                           0.0, 0.0, 2*drone3Params.inertiaMatrix.coeff(2,2)/(timeConst_Z*timeConst_Z);
        drone3Params.attCtrlTiltPrio.KD << 2*drone3Params.inertiaMatrix.coeff(0,0)*damping_X/timeConst_X, 0.0, 0.0,
                                           0.0, 2*drone3Params.inertiaMatrix.coeff(1,1)*damping_Y/timeConst_Y, 0.0,
                                           0.0, 0.0, 2*drone3Params.inertiaMatrix.coeff(2,2)*damping_Z/timeConst_Z;
        droneParams.push_back(drone3Params);
    }
}