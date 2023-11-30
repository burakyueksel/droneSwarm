/*
 * File: parameters.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "parameters.h"
#include <iostream> // for std::cout

namespace Parameters {
    std::vector<DroneParameters> droneParams;

    // helper functions:
    // compute bandwidth of the controller with a margin from actuators
    double computeOmega(double actTime, double margin)
    {
      return 1/(actTime*margin);
    }
    
    // compute p gain from the bandwdith of a 2nd order controller
    double computeKp(double omega)
    {
      return pow(omega,2);
    }

    // cpmpute d gain from the bandwidth and damping of a 2nd order controller
    double computeKd(double omega, double xi)
    {
      return 2*omega*xi;
    }

    // main function:
    // load the parameters.
    void loadParameters()
    {
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
      drone1Params.actBW_rps = 3.0; // Actuator bandwdith as radians per second
      // init states
      drone1Params.initPos << 0.0, 0.0, 0.0;
      // first, decide for the margins for stable cascaded control loop gains.
      // margin>1 means slower w.r.t. inner loop. At the core there is actuator bandwidth. Around it we build the control
      // loops with margins for stability and robustness.
      // TODO: roll and pitch are parametrized same, consider separating in case of different aicraft designs need it.
      // Attitude (rpy)
      double rpErrMargin   = 1.2; // tilt error margin makes it 20% slower than actuators can achieve
      double yawErrMargin  = 1.4; // make yaw slower than roll and pitch
      double rpRefMargin   = rpErrMargin*1.2;    // tilt reference is slower than roll error dymamics
      double yawRefMargin  = yawErrMargin*1.2;   // yaw reference is slower than yaw error dynamics
      // Altitude (Z)
      double altErrMargin  = 1.2; // altitude error margin makes it 20% slower than actuators can achieve
      double altRefMargin  = altErrMargin * 1.2; // make reference slower than yaw 
      // Position (XY)
      double posErrMargin = rpRefMargin * 2.0; // make it half slow as the attitude reference dynamics.
      double posRefMargin = posErrMargin * 2.0;// make it half slow as the position error dynamics.

      // now compute the gains
      // pos ctrl ref dyn gains
      double omega = computeOmega(drone1Params.actBW_rps,posRefMargin);//0.6; // make sure there is enough margin wrt inner loop
      double xi    = 1.0; // critically damped
      drone1Params.posCtrlRefDyn.Kp = computeKp(omega);
      drone1Params.posCtrlRefDyn.Kd = computeKd(omega, xi);
      // pos ctrl error
      omega = computeOmega(drone1Params.actBW_rps,posErrMargin);//1.0; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone1Params.posCtrlPID.Kp = computeKp(omega);
      drone1Params.posCtrlPID.Kd = computeKd(omega, xi);
      drone1Params.posCtrlPID.Ki = 0.1;
      // alt ctrl ref dyn
      omega = computeOmega(drone1Params.actBW_rps,altRefMargin);//0.8; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone1Params.altCtrlRefDyn.Kp = computeKp(omega);
      drone1Params.altCtrlRefDyn.Kd = computeKd(omega, xi);
      // alt pid ctrl err
      omega = computeOmega(drone1Params.actBW_rps,altErrMargin);//1.0; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone1Params.altCtrlPID.Kp = computeKp(omega);
      drone1Params.altCtrlPID.Kd = computeKd(omega, xi);
      drone1Params.altCtrlPID.Ki = 0.1;
      // att ctr ref dyn
      // att tilt prio ctrl err
      double omegaX = computeOmega(drone1Params.actBW_rps,rpErrMargin);//0.3;
      double omegaY = computeOmega(drone1Params.actBW_rps,rpErrMargin);//0.3;
      double omegaZ = computeOmega(drone1Params.actBW_rps,yawErrMargin);//0.3;
      double xiX    = 1.0;
      double xiY    = 1.0;
      double xiZ    = 1.0;
      drone1Params.attCtrlTiltPrio.KP << 2*drone1Params.inertiaMatrix.coeff(0,0)/(omegaX*omegaX), 0.0, 0.0,
                                          0.0, 2*drone1Params.inertiaMatrix.coeff(1,1)/(omegaY*omegaY), 0.0,
                                          0.0, 0.0, 2*drone1Params.inertiaMatrix.coeff(2,2)/(omegaZ*omegaZ);
      drone1Params.attCtrlTiltPrio.KD << 2*drone1Params.inertiaMatrix.coeff(0,0)*xiX/omegaX, 0.0, 0.0,
                                          0.0, 2*drone1Params.inertiaMatrix.coeff(1,1)*xiY/omegaY, 0.0,
                                          0.0, 0.0, 2*drone1Params.inertiaMatrix.coeff(2,2)*xiZ/omegaZ;
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
      drone2Params.inertiaMatrix << 1.0, 0.0, 0.0,
                                    0.0, 2.0, 0.0,
                                    0.0, 0.0, 3.0;
      drone2Params.actBW_rps = 3.0; // Actuator bandwdith as radians per second
      // init states
      drone2Params.initPos << 0.5, 0.0, 0.0;
      // first, decide for the margins for stable cascaded control loop gains.
      // margin>1 means slower w.r.t. inner loop. At the core there is actuator bandwidth. Around it we build the control
      // loops with margins for stability and robustness.
      // TODO: roll and pitch are parametrized same, consider separating in case of different aicraft designs need it.
      // Attitude (rpy)
      rpErrMargin   = 1.2; // tilt error margin makes it 20% slower than actuators can achieve
      yawErrMargin  = 1.4; // make yaw slower than roll and pitch
      rpRefMargin   = rpErrMargin*1.2;    // tilt reference is slower than roll error dymamics
      yawRefMargin  = yawErrMargin*1.2;   // yaw reference is slower than yaw error dynamics
      // Altitude (Z)
      altErrMargin  = 1.2; // altitude error margin makes it 20% slower than actuators can achieve
      altRefMargin  = altErrMargin * 1.2; // make reference slower than yaw 
      // Position (XY)
      posErrMargin = rpRefMargin * 2.0; // make it half slow as the attitude reference dynamics.
      posRefMargin = posErrMargin * 2.0;// make it half slow as the position error dynamics.

      // now compute the gains
      // pos ctrl ref dyn gains
      omega = computeOmega(drone2Params.actBW_rps,posRefMargin);//0.6; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone2Params.posCtrlRefDyn.Kp = computeKp(omega);
      drone2Params.posCtrlRefDyn.Kd = computeKd(omega, xi);
      // pos ctrl error
      omega = computeOmega(drone2Params.actBW_rps,posErrMargin);//1.0; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone2Params.posCtrlPID.Kp = computeKp(omega);
      drone2Params.posCtrlPID.Kd = computeKd(omega, xi);
      drone2Params.posCtrlPID.Ki = 0.1;
      // alt ctrl ref dyn
      omega = computeOmega(drone2Params.actBW_rps,altRefMargin);//0.8; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone2Params.altCtrlRefDyn.Kp = computeKp(omega);
      drone2Params.altCtrlRefDyn.Kd = computeKd(omega, xi);
      // alt pid ctrl err
      omega = computeOmega(drone2Params.actBW_rps,altErrMargin);//1.0; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone2Params.altCtrlPID.Kp = computeKp(omega);
      drone2Params.altCtrlPID.Kd = computeKd(omega, xi);
      drone2Params.altCtrlPID.Ki = 0.1;
      // att ctr ref dyn
      // att tilt prio ctrl err
      omegaX = computeOmega(drone2Params.actBW_rps,rpErrMargin);//0.3;
      omegaY = computeOmega(drone2Params.actBW_rps,rpErrMargin);//0.3;
      omegaZ = computeOmega(drone2Params.actBW_rps,yawErrMargin);//0.3;
      xiX    = 1.0;
      xiY    = 1.0;
      xiZ    = 1.0;
      drone2Params.attCtrlTiltPrio.KP << 2*drone2Params.inertiaMatrix.coeff(0,0)/(omegaX*omegaX), 0.0, 0.0,
                                          0.0, 2*drone2Params.inertiaMatrix.coeff(1,1)/(omegaY*omegaY), 0.0,
                                          0.0, 0.0, 2*drone2Params.inertiaMatrix.coeff(2,2)/(omegaZ*omegaZ);
      drone2Params.attCtrlTiltPrio.KD << 2*drone2Params.inertiaMatrix.coeff(0,0)*xiX/omegaX, 0.0, 0.0,
                                          0.0, 2*drone2Params.inertiaMatrix.coeff(1,1)*xiY/omegaY, 0.0,
                                          0.0, 0.0, 2*drone2Params.inertiaMatrix.coeff(2,2)*xiZ/omegaZ;

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
      drone2Params.traj4Gains.gains << 24.0, 50.0, 35.0, 10.0;

      // Compute the state feedback gains for 2 times differentiable (up to acceleration) trajectories
      // For the computation of the gains, see this file: /droneSwarm/analyse/computeGainsOf2ndOrderSmoother
      /* Following gains are computed for the following poles [-1.0, -2.0]
         with system matrix and input matrices are as in the following:
         A = np.array([[0, 1],
                     [0, 0]])

         B = np.array([[0],
                     [1]])
      */
      drone2Params.traj2Gains.gains << 2.0, 3.0;
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
      drone3Params.actBW_rps = 3.0; // Actuator bandwdith as radians per second
      // init states
      drone3Params.initPos <<1.0, 0.0, 0.0;
      // first, decide for the margins for stable cascaded control loop gains.
      // margin>1 means slower w.r.t. inner loop. At the core there is actuator bandwidth. Around it we build the control
      // loops with margins for stability and robustness.
      // TODO: roll and pitch are parametrized same, consider separating in case of different aicraft designs need it.
      // Attitude (rpy)
      rpErrMargin   = 1.2; // tilt error margin makes it 20% slower than actuators can achieve
      yawErrMargin  = 1.4; // make yaw slower than roll and pitch
      rpRefMargin   = rpErrMargin*1.2;    // tilt reference is slower than roll error dymamics
      yawRefMargin  = yawErrMargin*1.2;   // yaw reference is slower than yaw error dynamics
      // Altitude (Z)
      altErrMargin  = 1.2; // altitude error margin makes it 20% slower than actuators can achieve
      altRefMargin  = altErrMargin * 1.2; // make reference slower than yaw 
      // Position (XY)
      posErrMargin = rpRefMargin * 2.0; // make it half slow as the attitude reference dynamics.
      posRefMargin = posErrMargin * 2.0;// make it half slow as the position error dynamics.

      // now compute the gains
      // pos ctrl ref dyn gains
      omega = computeOmega(drone3Params.actBW_rps,posRefMargin);//0.6; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone3Params.posCtrlRefDyn.Kp = computeKp(omega);
      drone3Params.posCtrlRefDyn.Kd = computeKd(omega, xi);
      // pos ctrl error
      omega = computeOmega(drone3Params.actBW_rps,posErrMargin);//1.0; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone3Params.posCtrlPID.Kp = computeKp(omega);
      drone3Params.posCtrlPID.Kd = computeKd(omega, xi);
      drone3Params.posCtrlPID.Ki = 0.1;
      // alt ctrl ref dyn
      omega = computeOmega(drone3Params.actBW_rps,altRefMargin);//0.8; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone3Params.altCtrlRefDyn.Kp = computeKp(omega);
      drone3Params.altCtrlRefDyn.Kd = computeKd(omega, xi);
      // alt pid ctrl err
      omega = computeOmega(drone3Params.actBW_rps,altErrMargin);//1.0; // make sure there is enough margin wrt inner loop
      xi    = 1.0; // critically damped
      drone3Params.altCtrlPID.Kp = computeKp(omega);
      drone3Params.altCtrlPID.Kd = computeKd(omega, xi);
      drone3Params.altCtrlPID.Ki = 0.1;
      // att ctr ref dyn
      // att tilt prio ctrl err
      omegaX = computeOmega(drone3Params.actBW_rps,rpErrMargin);//0.3;
      omegaY = computeOmega(drone3Params.actBW_rps,rpErrMargin);//0.3;
      omegaZ = computeOmega(drone3Params.actBW_rps,yawErrMargin);//0.3;
      xiX    = 1.0;
      xiY    = 1.0;
      xiZ    = 1.0;
      drone3Params.attCtrlTiltPrio.KP << 2*drone3Params.inertiaMatrix.coeff(0,0)/(omegaX*omegaX), 0.0, 0.0,
                                          0.0, 2*drone3Params.inertiaMatrix.coeff(1,1)/(omegaY*omegaY), 0.0,
                                          0.0, 0.0, 2*drone3Params.inertiaMatrix.coeff(2,2)/(omegaZ*omegaZ);
      drone3Params.attCtrlTiltPrio.KD << 2*drone3Params.inertiaMatrix.coeff(0,0)*xiX/omegaX, 0.0, 0.0,
                                          0.0, 2*drone3Params.inertiaMatrix.coeff(1,1)*xiY/omegaY, 0.0,
                                          0.0, 0.0, 2*drone3Params.inertiaMatrix.coeff(2,2)*xiZ/omegaZ;
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
      drone3Params.traj4Gains.gains << 24.0, 50.0, 35.0, 10.0;

      // Compute the state feedback gains for 2 times differentiable (up to acceleration) trajectories
      // For the computation of the gains, see this file: /droneSwarm/analyse/computeGainsOf2ndOrderSmoother
      /* Following gains are computed for the following poles [-1.0, -2.0]
         with system matrix and input matrices are as in the following:
         A = np.array([[0, 1],
                     [0, 0]])

         B = np.array([[0],
                     [1]])
      */
      drone3Params.traj2Gains.gains << 2.0, 3.0;
      droneParams.push_back(drone3Params);
    }
}