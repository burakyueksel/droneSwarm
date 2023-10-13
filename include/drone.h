/**
 * @file drone.h
 * @brief This file contains the declerations of all functions used for a drone.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations
#include "parameters.h"

class Drone
{
public:
    /**
     * @brief Represents a drone in the drone swarm.
     */
    Drone(int id); /**< The ID of the drone. */
    /**
     * Updates the state of the drone based on the given time step.
     * This function computes the new position, orientation, and velocity of the drone.
     *
     * @param timeStep The time step for the state update.
     */
    void updateState(double timeStep);
    void setExternalForceBody(const Eigen::Vector3d& force); // Set the value of externalForceBody
    void setExternalTorqueBody(const Eigen::Vector3d& torque); // Set the value of externalTorqueBody
    /**
     * Gets the ID of the drone.
     *
     * @return The ID of the drone.
     */
    int getID() const;
    trajectorySE3 trajectoryGenSE3(flatOutputsSE3 flatCmd, double timeStep_s);
    posCtrlRefStates posControlRefDyn(horizontalStates posCmd, double timeStep_s);
    altCtrlRefStates altControlRefDyn(double zCmd, double timeStep_s);
    horizontalStates posCtrlErr(posCtrlRefStates posRefStates, Eigen::Vector3d position, Eigen::Vector3d velocity, double timeStep_s);
    Eigen::Quaterniond attTiltPrioRefDyn(double ddxRef, double ddyRef, double ddzRef, double des_yaw_rad);
    altCtrlErrOutputs altPidControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, double timeStep_s); // helper function for altitude control
    Eigen::Vector3d attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps);
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Quaterniond getQuaternion() const;
    Eigen::Vector3d getBodyRates() const;
    Eigen::Quaterniond eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg);
    /*
    posCtrlRefStates getPosCtrlRefStates() const;
    altCtrlRefStates getAltCtrlRefStates() const;
    */
    // Add more member functions as needed

private:
    int id;
    double g_altIntegral;
    trajectorySE3 g_trajectorySE3;
    horizontalStates g_horizontalPosIntegral;
    posCtrlRefStates g_posCtrlRefDynStates;
    altCtrlRefStates g_altCtrlRefDynStates;
    DroneParameters parameters;
    /// The position of the drone.
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angularVelocity; // Angular velocity as a member variable
    Eigen::Quaterniond orientation; // Orientation as a member variable
    Eigen::Vector3d externalTorqueBody; // External torque in body frame as a member variable
    Eigen::Vector3d externalForceBody; // External force in body frame as a member variable
    // Add other relevant private member variables

    // Define other relevant private member variables
};
