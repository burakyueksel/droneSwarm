/*
 * File: drone.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "drone.h"
#include <iostream>

Drone::Drone(int id)
{
    // Set the drone ID
    this->id = id;

    // Initialize the parameters based on the ID
    if (id >= 0 && id < Parameters::droneParams.size()) {
        parameters = Parameters::droneParams[id];
    } else {
        // Handle the case when the ID is out of range
        std::cout << "Error: Invalid drone ID!" << std::endl;
        // You may choose to assign default parameters or handle the error in another way
    }
    // Initialize the member variables
    velocity.setZero();
    position.setZero();
    externalForce.setZero();
    angularVelocity.setZero();
    orientation.setIdentity();
    externalTorque.setZero();
}


void Drone::updateState(double timeStep) {
    // Update the drone's state based on dynamics

    // Update translational dynamics:
    // Compute translational acceleration
    // Compute gravity force
    Eigen::Vector3d gravityForce(0.0, 0.0, -9.81 * parameters.mass);

    // Compute net force
    Eigen::Vector3d netForce = externalForce + gravityForce;

    // Compute acceleration
    Eigen::Vector3d acceleration = netForce / parameters.mass;
    // Update velocity and position
    position = position + velocity * timeStep + 0.5 * acceleration * pow(timeStep,2) ;
    velocity += acceleration * timeStep;

    // Update rotational dynamics:
    // Compute angular momentum
    Eigen::Vector3d angularMomentum = angularVelocity.cross(parameters.inertiaMatrix * angularVelocity);
    // Calculate the angular acceleration
    Eigen::Vector3d angularAcceleration  = parameters.inertiaMatrix.inverse() * (externalTorque-angularMomentum);

    // Integrate the angular acceleration to update the angular velocity
    angularVelocity += angularAcceleration * timeStep;

    // Compute q_dot from q and angular velocity. Then integrate q_dot to q.

    // Update other state variables (position, velocity, etc.) as needed

}

// external torques: control torques, disturbance torques, etc
void Drone::setExternalTorque(const Eigen::Vector3d& torque) {
    externalTorque = torque;
}

// external forces: control forces, disturbance forces, etc
void Drone::setExternalForce(const Eigen::Vector3d& force) {
    externalForce = force;
}

int Drone::getID() const {
    return id;
}

Eigen::Vector3d Drone::getPosition() const {
    return position;
}