/*
 * File: drone.h
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */

#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations
#include "parameters.h"

class Drone
{
public:
    Drone(int id); // Constructor with drone ID
    void updateState(double timeStep); // Update the drone's state based on dynamics
    void setExternalForceBody(const Eigen::Vector3d& force); // Set the value of externalForceBody
    void setExternalTorqueBody(const Eigen::Vector3d& torque); // Set the value of externalTorqueBody
    int getID() const;
    posCtrlStates posControlRefDyn(Eigen::Vector2d posCmd, double timeStep_s);
    Eigen::Vector3d altControlRefDyn(double zCmd, double timeStep_s);
    double altPidControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, double timeStep_s); // helper function for altitude control
    Eigen::Vector3d attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps);
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Quaterniond getQuaternion() const;
    Eigen::Vector3d getBodyRates() const;
    // Add more member functions as needed

private:
    int id;
    double g_altIntegral;
    posCtrlStates g_posCtrlRefDynStates;
    Eigen::Vector3d g_altCtrlRefDynStates;
    DroneParameters parameters;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angularVelocity; // Angular velocity as a member variable
    Eigen::Quaterniond orientation; // Orientation as a member variable
    Eigen::Vector3d externalTorqueBody; // External torque in body frame as a member variable
    Eigen::Vector3d externalForceBody; // External force in body frame as a member variable
    // Add other relevant private member variables

    // Define other relevant private member variables
};
