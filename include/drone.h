/*
 * File: drone.h
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */

#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

class Drone
{
public:
    Drone(); // Constructor
    void updateState(double timeStep); // Update the drone's state based on dynamics
    // Add more member functions as needed

private:
    // Define the drone's state variables (position, velocity, orientation as quaternions, etc.)
    // For example:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angularVelocity; // Angular velocity as a member variable
    Eigen::Quaterniond orientation; // Orientation as a member variable
    // Add other relevant private member variables

    // Define other relevant private member variables
};
