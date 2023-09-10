/*
 * File: parameters.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "parameters.h"
#include <iostream> // for std::cout

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
                                      0.0, 0.0, 2.0;
        // init states
        drone1Params.initPos << 0.0, 0.0, 0.0;
        // pos ctrl ref dyn
        drone1Params.posCtrlRefDyn.timeConst = 0.6; // make sure there is enough margin wrt inner loop
        drone1Params.posCtrlRefDyn.damping   = 1.0; // critically damped
        // pos ctrl error
        double posOmega1 = 1.0; // make sure there is enough margin wrt inner loop
        double posXi1    = 1.0;
        drone1Params.posCtrlPID.Kp = pow(posOmega1,2);
        drone1Params.posCtrlPID.Ki = 0.1;
        drone1Params.posCtrlPID.Kd = 2*posOmega1*posXi1;
        // alt ctrl ref dyn
        drone1Params.altCtrlRefDyn.timeConst = 0.8; // make sure there is enough margin wrt inner loop
        drone1Params.altCtrlRefDyn.damping   = 1.0; // critically damped
        // alt pid ctrl
        double omega1 = 3;
        double xi1 = 1;
        drone1Params.altCtrlPID.Kp = pow(omega1,2);
        drone1Params.altCtrlPID.Ki = 0.1;
        drone1Params.altCtrlPID.Kd = 2*omega1*xi1;
        // att tilt prio ctrl
        double timeConst1_X = 0.3;
        double timeConst1_Y = 0.3;
        double timeConst1_Z  = 0.3;
        double damping1_X = 0.707;
        double damping1_Y = 0.707;
        double damping1_Z = 0.707;
        drone1Params.attCtrlTiltPrio.KP << 2*drone1Params.inertiaMatrix.coeff(0,0)/(timeConst1_X*timeConst1_X), 0.0, 0.0,
                                           0.0, 2*drone1Params.inertiaMatrix.coeff(1,1)/(timeConst1_Y*timeConst1_Y), 0.0,
                                           0.0, 0.0, 2*drone1Params.inertiaMatrix.coeff(2,2)/(timeConst1_Z*timeConst1_Z);
        drone1Params.attCtrlTiltPrio.KD << 2*drone1Params.inertiaMatrix.coeff(0,0)*damping1_X/timeConst1_X, 0.0, 0.0,
                                           0.0, 2*drone1Params.inertiaMatrix.coeff(1,1)*damping1_Y/timeConst1_Y, 0.0,
                                           0.0, 0.0, 2*drone1Params.inertiaMatrix.coeff(2,2)*damping1_Z/timeConst1_Z;
        // trajectory generation

        // Compute the state feedback gains for 4 times differentiable (up to snap) trajectories
        // For the computation of the gains, see this file: /droneSwarm/analyse/computeGainsOf4thOrderSmoother
        /* Following gains are computed for the following poles [-1.0, -2, -3.0, -4.0]
           with system matrix and input matrices are as in the following:
           A = np.array([[0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

            B = np.array([[0],
                        [0],
                        [0],
                        [1]])
        */
        drone1Params.traj4Gains.gains << 24.0, 50.0, 35.0, 10.0;

        // Compute the state feedback gains for 2 times differentiable (up to acceleration) trajectories
        // For the computation of the gains, see this file: /droneSwarm/analyse/computeGainsOf2ndOrderSmoother
        /* Following gains are computed for the following poles [-1.0, -2.0]
           with system matrix and input matrices are as in the following:
            A = np.array([[0, 1],
                        [0, 0]])

            B = np.array([[0],
                        [1]])
        */
        drone1Params.traj2Gains.gains << 2.0, 3.0;
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
        drone2Params.inertiaMatrix << 1.0, 0.0, 0.0,
                                      0.0, 2.0, 0.0,
                                      0.0, 0.0, 3.0;
        // init states
        drone2Params.initPos << 0.5, 0.0, 0.0;
        // pos ctrl ref dyn
        drone2Params.posCtrlRefDyn.timeConst = 0.6; // make sure there is enough margin wrt inner loop
        drone2Params.posCtrlRefDyn.damping   = 1.0; // critically damped
        // pos ctrl error
        double posOmega2 = 1.0; // make sure there is enough margin wrt inner loop
        double posXi2    = 1.0;
        drone2Params.posCtrlPID.Kp = pow(posOmega2,2);
        drone2Params.posCtrlPID.Ki = 0.1;
        drone2Params.posCtrlPID.Kd = 2*posOmega2*posXi2;
        // alt ctrl ref dyn
        drone2Params.altCtrlRefDyn.timeConst = 0.8; // make sure there is enough margin wrt inner loop
        drone2Params.altCtrlRefDyn.damping   = 0.707; // under damped
        // alt pid ctrl
        double omega2 = 3;
        double xi2 = 1;
        drone2Params.altCtrlPID.Kp = pow(omega2,2);
        drone2Params.altCtrlPID.Ki = 0.1;
        drone2Params.altCtrlPID.Kd = 2*omega2*xi2;
        // att tilt prio ctrl
        double timeConst2_X = 0.3;
        double timeConst2_Y = 0.3;
        double timeConst2_Z  = 0.3;
        double damping2_X = 0.707;
        double damping2_Y = 0.707;
        double damping2_Z = 0.707;
        drone2Params.attCtrlTiltPrio.KP << 2*drone2Params.inertiaMatrix.coeff(0,0)/(timeConst2_X*timeConst2_X), 0.0, 0.0,
                                           0.0, 2*drone2Params.inertiaMatrix.coeff(1,1)/(timeConst2_Y*timeConst2_Y), 0.0,
                                           0.0, 0.0, 2*drone2Params.inertiaMatrix.coeff(2,2)/(timeConst2_Z*timeConst2_Z);
        drone2Params.attCtrlTiltPrio.KD << 2*drone2Params.inertiaMatrix.coeff(0,0)*damping2_X/timeConst2_X, 0.0, 0.0,
                                           0.0, 2*drone2Params.inertiaMatrix.coeff(1,1)*damping2_Y/timeConst2_Y, 0.0,
                                           0.0, 0.0, 2*drone2Params.inertiaMatrix.coeff(2,2)*damping2_Z/timeConst2_Z;
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
        drone3Params.mass = 1.0; // Set the mass for drone 3
        drone3Params.inertiaMatrix << 1.0, 0.0, 0.0,
                                      0.0, 2.0, 0.0,
                                      0.0, 0.0, 4.0;
        // init states
        drone3Params.initPos <<1.0, 0.0, 0.0;
        // pos ctrl ref dyn
        drone3Params.posCtrlRefDyn.timeConst = 0.8; // make sure there is enough margin wrt inner loop
        drone3Params.posCtrlRefDyn.damping   = 1.0; // critically damped
        // pos ctrl error
        double posOmega3 = 1.0; // make sure there is enough margin wrt inner loop
        double posXi3    = 1.0;
        drone3Params.posCtrlPID.Kp = pow(posOmega3,2);
        drone3Params.posCtrlPID.Ki = 0.1;
        drone3Params.posCtrlPID.Kd = 2*posOmega3*posXi3;
        // alt ctrl ref dyn
        drone3Params.altCtrlRefDyn.timeConst = 1.0; // make sure there is enough margin wrt inner loop
        drone3Params.altCtrlRefDyn.damping   = 1.0; // critically damped
        // alt pid ctrl
        double omega3 = 3;
        double xi3 = 1;
        drone3Params.altCtrlPID.Kp = pow(omega3,2);
        drone3Params.altCtrlPID.Ki = 0.1;
        drone3Params.altCtrlPID.Kd = 2*omega3*xi3;
        // att tilt prio ctrl
        double timeConst3_X = 0.3;
        double timeConst3_Y = 0.3;
        double timeConst3_Z  = 0.3;
        double damping3_X = 0.707;
        double damping3_Y = 0.707;
        double damping3_Z = 0.707;
        drone3Params.attCtrlTiltPrio.KP << 2*drone3Params.inertiaMatrix.coeff(0,0)/(timeConst3_X*timeConst3_X), 0.0, 0.0,
                                           0.0, 2*drone3Params.inertiaMatrix.coeff(1,1)/(timeConst3_Y*timeConst3_Y), 0.0,
                                           0.0, 0.0, 2*drone3Params.inertiaMatrix.coeff(2,2)/(timeConst3_Z*timeConst3_Z);
        drone3Params.attCtrlTiltPrio.KD << 2*drone3Params.inertiaMatrix.coeff(0,0)*damping3_X/timeConst3_X, 0.0, 0.0,
                                           0.0, 2*drone3Params.inertiaMatrix.coeff(1,1)*damping3_Y/timeConst3_Y, 0.0,
                                           0.0, 0.0, 2*drone3Params.inertiaMatrix.coeff(2,2)*damping3_Z/timeConst3_Z;
        droneParams.push_back(drone3Params);
    }
}