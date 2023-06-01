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
    void setExternalForce(const Eigen::Vector3d& force); // Set the value of externalForce
    void setExternalTorque(const Eigen::Vector3d& torque); // Set the value of externalTorque
    int getID() const;
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;

    // Add more member functions as needed

private:
    int id;
    DroneParameters parameters;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angularVelocity; // Angular velocity as a member variable
    Eigen::Quaterniond orientation; // Orientation as a member variable
    Eigen::Vector3d externalTorque; // External torque as a member variable
    Eigen::Vector3d externalForce; // External force as a member variable
    // Add other relevant private member variables

    // Define other relevant private member variables
};
