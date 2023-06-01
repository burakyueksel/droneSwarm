/*
 * File: drone.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "drone.h"

Drone::Drone()
{
    // Initialize the member variables
    position.setZero();
    velocity.setZero();
    angularVelocity.setZero();
    orientation.setIdentity();
    externalTorque.setZero();
}


void Drone::updateState(double timeStep) {
    // Update the drone's state based on dynamics

    // Update translational dynamics (position, velocity, etc.) as per your requirements

    // Update rotational dynamics using quaternions
    Eigen::Matrix3d inertiaMatrix; // Define the inertia matrix

    // Calculate the angular acceleration based on the second-order quaternion dynamics equation
    Eigen::Vector3d angularAcceleration; // here compute the angular acceleration

    // Integrate the angular acceleration to update the angular velocity
    angularVelocity += angularAcceleration * timeStep;

    // Compute q_dot from q and angular velocity. Then integrate q_dot to q.

    // Update other state variables (position, velocity, etc.) as needed

}


void Drone::setExternalTorque(const Eigen::Vector3d& torque) {
    externalTorque = torque;
}