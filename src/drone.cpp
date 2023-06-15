/*
 * File: drone.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "drone.h"
#include "environment.h"
#include <iostream>

Drone::Drone(int id)
{
    // Set the drone ID
    this->id = id;

    // Initialize the parameters based on the ID
    if (id >= 0 && id < Parameters::droneParams.size())
    {
        parameters = Parameters::droneParams[id];
        position   = parameters.initPos;
        // Print the parameters used for this drone
        std::cout << "Drone ID: " << id << std::endl;
        std::cout << "Mass: " << parameters.mass << std::endl;
        std::cout << "Inertia Matrix:\n" << parameters.inertiaMatrix << std::endl;
    }
    else
    {
        // Handle the case when the ID is out of range
        std::cout << "Error: Invalid drone ID!" << std::endl;
        // You may choose to assign default parameters or handle the error in another way
    }
    // Initialize the member variables
    velocity.setZero();
    //position.setZero();
    externalForce.setZero();
    angularVelocity.setZero();
    orientation.setIdentity();
    externalTorque.setZero();
}


void Drone::updateState(double timeStep) {
    // Update the drone's state based on dynamics

    // Update translational dynamics:
    // Compute translational acceleration
    // Compute gravity force (NED reference frame)
    Eigen::Vector3d gravityForce(0.0, 0.0, Environment::GRAVITY * parameters.mass);

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
    // Convert angular velocity to the time derivative of quaternion
    // source: https://ahrs.readthedocs.io/en/latest/filters/angular.html
    // source: https://github.com/burakyueksel/physics/blob/eeba843fe20e5fd4e2d5d2d3d9608ed038bfb069/src/physics.c#L93
    Eigen::Quaterniond orientationDot;
    orientationDot.w() = -0.5  * (angularVelocity.x() * orientation.x() + angularVelocity.y() * orientation.y() + angularVelocity.z() * orientation.z());
    orientationDot.x() =  0.5  * (angularVelocity.x() * orientation.w() + angularVelocity.z() * orientation.y() - angularVelocity.y() * orientation.z());
    orientationDot.y() =  0.5  * (angularVelocity.y() * orientation.w() - angularVelocity.z() * orientation.x() + angularVelocity.x() * orientation.z());
    orientationDot.z() =  0.5  * (angularVelocity.z() * orientation.w() + angularVelocity.y() * orientation.x() - angularVelocity.x() * orientation.y());
    // Integrate orientationDot with time step
    orientation.w() += orientationDot.w() * timeStep;
    orientation.x() += orientationDot.x() * timeStep;
    orientation.y() += orientationDot.y() * timeStep;
    orientation.z() += orientationDot.z() * timeStep;
    orientation.normalize();  // Normalize the quaternion
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

Eigen::Vector3d Drone::getVelocity() const {
    return velocity;
}
Eigen::Quaterniond Drone::getQuaternion() const {
    return orientation;
}

Eigen::Vector3d Drone::getBodyRates() const {
    return angularVelocity;
}

//  Altitude PID control
double Drone::altPidControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, double timeStep_s)
{
    // error
    double error = zDes_m - z_m;

    // d_error
    double d_error = dzDes_mps - dz_mps;

    // Proportional term
    double proportional = parameters.altCtrlPID.Kp * error;

    // Integral term
    altIntegral += parameters.altCtrlPID.Ki * error * timeStep_s;

    // todo: add proper anti-windup

    // Derivative term
    double derivative = parameters.altCtrlPID.Kd * d_error;

    // Calculate the thrust for height control
    double controlThrust_N = proportional + altIntegral + derivative;

    return controlThrust_N;
}

// Attitude, tilt prioritizing quaternion based control
Eigen::Vector3d Drone::attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps)
{
    // source:https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf

    // eq.13
    Eigen::Quaterniond quatError = quatDes * quat.inverse();
    // eq. 14
    Eigen::Vector3d angVelErr_rps = angVelDes_rps - angVel_rps;
    // compute 1/sqrt(quat.w² + quat.z²)
    double qNorm = quatError.w()*quatError.w() + quatError.z()*quatError.z();
    double oneOverQuatErrRedNorm;
    // in case quat isn not well defined
    // this happens in the following case as an example:
    // quat is the quaternion between a desired frame and the current frame
    // z axis of the desired frame is aligned exactly at the opposite direction of the z axis of the current frame
    if (qNorm<1e-6)
    {
        oneOverQuatErrRedNorm = 0.001; // a small number.
    }
    else
    {
        oneOverQuatErrRedNorm = 1/sqrtf(qNorm);
    }
    // eq. 18
    Eigen::Quaterniond quatErrRed;
    quatErrRed.w() = oneOverQuatErrRedNorm * (quatError.w()*quatError.w() + quatError.z()*quatError.z());
    quatErrRed.x() = oneOverQuatErrRedNorm * (quatError.w()*quatError.x() - quatError.y()*quatError.z());
    quatErrRed.y() = oneOverQuatErrRedNorm * (quatError.w()*quatError.y() + quatError.x()*quatError.z());
    quatErrRed.z() = 0.0;
    // eq. 20
    Eigen::Quaterniond quatErrYaw;
    quatErrYaw.w() = oneOverQuatErrRedNorm * quatError.w();
    quatErrYaw.x() = 0.0;
    quatErrYaw.y() = 0.0;
    quatErrYaw.z() = oneOverQuatErrRedNorm * quatError.z();
    // eq. 23
    
}