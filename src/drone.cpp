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
    angularVelocity.setZero();
    orientation.setIdentity();
    externalForceBody.setZero();
    externalTorqueBody.setZero();
}

template <typename T>
int mySignum(T value) {
    return (value > T(0)) - (value < T(0));
}

Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& q)
{
    // Convert quaternion to rotation matrix
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    // Define Euler angle vector
    Eigen::Vector3d eulerAngles;

    // Extract roll, pitch, and yaw angles from rotation matrix
    eulerAngles.x() = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    eulerAngles.y() = asin(-rotationMatrix(2, 0));
    eulerAngles.z() = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

    return eulerAngles;
}

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q)
{
    // Convert quaternion to rotation matrix
    // This rotation represents the orientation of the body frame w.r.t. an inertial frame.
    // Meaning, it can left multiply a vector represented in body frame, to bring it to the inertial frame
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;
}

Eigen::Vector3d quat2Re3(const Eigen::Quaterniond& q)
{
    // Compute Re3, that is the third column of the rotation matrix (from body to inertial frame)
    Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    Eigen::Vector3d Re3 = R.col(2);

    return Re3;
}

Eigen::Vector3d quat2RTe3(const Eigen::Quaterniond& q)
{
    // Compute R^Te3, that is the third column of the transpose of the rotation matrix (from inertial to body frame)
    Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    Eigen::Matrix3d RT = R.transpose();
    Eigen::Vector3d RTe3 = RT.col(2);

    return RTe3;
}

double quat2R33(const Eigen::Quaterniond& q)
{
    // compute the (3,3)th element of the rotation matrix from quaternion
    Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    double minThreshold = 1e-4;
    // protect it for very small numbers (we tend to use this value for division)
    double R33 = std::abs(R.coeff(2, 2)) < minThreshold ? minThreshold : R.coeff(2, 2);

    return R33;
}


/*
Implements eq 1 of https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
*/
Eigen::Quaterniond angleAxisToQuaternion (const double& angle, const Eigen::Vector3d vector)
{
    // define unit quaternion
    Eigen::Quaterniond quat;
    // trigonometric constants
    double ca2 = cos(angle/2);
    double sa2 = sin(angle/2);
    quat.w() = ca2;
    quat.x() = vector.x()*sa2;
    quat.y() = vector.y()*sa2;
    quat.z() = vector.z()*sa2;

    return quat;
}

Eigen::Quaterniond Drone::eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
    // Convert roll, pitch, and yaw angles to radians
    double rollRad = roll_deg * Environment::PI / 180.0;
    double pitchRad = pitch_deg * Environment::PI / 180.0;
    double yawRad = yaw_deg * Environment::PI / 180.0;

    // Calculate half angles
    double cosRollHalf = cos(rollRad / 2.0);
    double sinRollHalf = sin(rollRad / 2.0);
    double cosPitchHalf = cos(pitchRad / 2.0);
    double sinPitchHalf = sin(pitchRad / 2.0);
    double cosYawHalf = cos(yawRad / 2.0);
    double sinYawHalf = sin(yawRad / 2.0);

    // Compute quaternion components
    double w = cosRollHalf * cosPitchHalf * cosYawHalf + sinRollHalf * sinPitchHalf * sinYawHalf;
    double x = sinRollHalf * cosPitchHalf * cosYawHalf - cosRollHalf * sinPitchHalf * sinYawHalf;
    double y = cosRollHalf * sinPitchHalf * cosYawHalf + sinRollHalf * cosPitchHalf * sinYawHalf;
    double z = cosRollHalf * cosPitchHalf * sinYawHalf - sinRollHalf * sinPitchHalf * cosYawHalf;

    // Return the quaternion
    return Eigen::Quaterniond(w, x, y, z);
}

void Drone::updateState(double timeStep) {
    // Update the drone's state based on dynamics

    // Update translational dynamics:
    // Translational dynamics are evolving in inertial frame
    // Compute translational acceleration
    // Compute gravity force (NED reference frame)
    Eigen::Vector3d gravityForce(0.0, 0.0, Environment::GRAVITY * parameters.mass);

    // Get the orientation as rotation matrix
    Eigen::Matrix3d rotMat = quaternionToRotationMatrix(orientation);

    // Compute net force in inertial frame
    Eigen::Vector3d netForce = rotMat * externalForceBody + gravityForce;

    // Compute acceleration
    Eigen::Vector3d acceleration = netForce / parameters.mass;
    // Update velocity and position
    position = position + velocity * timeStep + 0.5 * acceleration * pow(timeStep,2) ;
    velocity += acceleration * timeStep;

    // Update rotational dynamics:
    // Rotational dynamics are evolving in body frame
    // Compute angular momentum
    Eigen::Vector3d angularMomentum = angularVelocity.cross(parameters.inertiaMatrix * angularVelocity);
    // Calculate the angular acceleration
    Eigen::Vector3d angularAcceleration  = parameters.inertiaMatrix.inverse() * (externalTorqueBody-angularMomentum);
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
void Drone::setExternalTorqueBody(const Eigen::Vector3d& torque) {
    externalTorqueBody = torque;
}

// external forces: control forces, disturbance forces, etc
void Drone::setExternalForceBody(const Eigen::Vector3d& force) {
    externalForceBody = force;
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

/*
posCtrlRefStates Drone::getPosCtrlRefStates() const {
    return g_posCtrlRefDynStates;
}

altCtrlRefStates Drone::getAltCtrlRefStates() const {
    return g_altCtrlRefDynStates;
}
*/
posCtrlRefStates Drone::posControlRefDyn(horizontalStates posCmd, double timeStep_s)
{
    double timeConst = parameters.posCtrlRefDyn.timeConst;
    double damping   = parameters.posCtrlRefDyn.damping;

    double posErrorX = posCmd.x - g_posCtrlRefDynStates.posRef.x;
    double posErrorY = posCmd.y - g_posCtrlRefDynStates.posRef.y;

    g_posCtrlRefDynStates.accRef.x = posErrorX * timeConst * timeConst - 2.0 * damping * timeConst * g_posCtrlRefDynStates.velRef.x;
    g_posCtrlRefDynStates.accRef.y = posErrorY * timeConst * timeConst - 2.0 * damping * timeConst * g_posCtrlRefDynStates.velRef.y;
    // TODO: add acc limits
    // integrate to velocity now
    g_posCtrlRefDynStates.velRef.x = g_posCtrlRefDynStates.velRef.x + g_posCtrlRefDynStates.accRef.x * timeStep_s;
    g_posCtrlRefDynStates.velRef.y = g_posCtrlRefDynStates.velRef.y + g_posCtrlRefDynStates.accRef.y * timeStep_s;
    // TODO: add vel limits
    // integrate to position now
    g_posCtrlRefDynStates.posRef.x = g_posCtrlRefDynStates.posRef.x + g_posCtrlRefDynStates.velRef.x * timeStep_s;
    g_posCtrlRefDynStates.posRef.y = g_posCtrlRefDynStates.posRef.y + g_posCtrlRefDynStates.velRef.y * timeStep_s;

    return g_posCtrlRefDynStates;
}

// Position Error Dynamics
horizontalStates Drone::posCtrlErr(posCtrlRefStates posRefStates, Eigen::Vector3d position, Eigen::Vector3d velocity, double timeStep_s)
{
    // error
    double errorX = posRefStates.posRef.x - position[0];
    double errorY = posRefStates.posRef.x - position[1];

    // d_error
    double derrorX = posRefStates.velRef.x - velocity[0];
    double derrorY = posRefStates.velRef.y - velocity[1];

    // Proportional term
    double proportionalX = parameters.posCtrlPID.Kp * errorX;
    double proportionalY = parameters.posCtrlPID.Kp * errorY;

    // Integral term
    g_horizontalPosIntegral.x += parameters.posCtrlPID.Ki * errorX * timeStep_s;
    g_horizontalPosIntegral.y += parameters.posCtrlPID.Ki * errorY * timeStep_s;

    // todo: add proper anti-windup

    // Derivative term
    double derivativeX = parameters.posCtrlPID.Kd * derrorX;
    double derivativeY = parameters.posCtrlPID.Kd * derrorY;

    // Calculate the desired accelerations in x and y axis
    horizontalStates accXYRef;
    accXYRef.x = posRefStates.accRef.x + proportionalX + g_horizontalPosIntegral.x + derivativeX;
    accXYRef.y = posRefStates.accRef.y + proportionalY + g_horizontalPosIntegral.y + derivativeY;

    return accXYRef;
}

// Attitude control reference dynamics loop, gives desired attitude in quaternion from position errors.
Eigen::Quaterniond Drone::attTiltPrioRefDyn(double ddxCmd, double ddyCmd, double ddzCmd, double des_yaw_rad)
{
    //source:https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf

    // Acc ref could have come in as acc cmd, if the desired positions are tracked perfectly.
    // Clearly this is not the case. Another position error ctrl loop needs to be added between the pos reference dynamics and this att tracker

    double ddxyNorm = sqrt(ddxCmd*ddxCmd + ddyCmd*ddyCmd);
    double minThreshold = 1e-4;
    // protect it for very small numbers (we tend to use this value for division)
    ddxyNorm = std::abs(ddxyNorm) < minThreshold ? minThreshold : ddxyNorm;

    // eq. 50
    double angle_red = atan2(ddxyNorm, ddzCmd - Environment::GRAVITY);
    // eq. 51
    Eigen::Vector3d vector_red (-ddyCmd/ddxyNorm, ddxCmd/ddxyNorm, 0);

    Eigen::Quaterniond quat_red = angleAxisToQuaternion(angle_red, vector_red);
    Eigen::Quaterniond quat_des_yaw (cos(des_yaw_rad/2), 0, 0, sin(des_yaw_rad/2));
    // eq. 52
    Eigen::Quaterniond quat_des = quat_des_yaw * quat_red;

    return quat_des;
}
// Attitude, tilt prioritizing quaternion based control
Eigen::Vector3d Drone::attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps)
{
    // source:https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf

    // eq.13
    Eigen::Quaterniond quatError = quatDes * quat.inverse(); // assumption: eigen does the correct multiplication
    // eq. 14
    Eigen::Vector3d angVelErr_rps = angVelDes_rps - angVel_rps;
    // compute 1/sqrt(quat.w² + quat.z²)
    double qNorm = quatError.w()*quatError.w() + quatError.z()*quatError.z();
    double oneOverQuatErrRedNorm;
    // in case quat isn not well defined
    // this happens in the following case as an example:
    // quat is the quaternion between a desired frame and the current frame
    // z axis of the desired frame is aligned exactly at the opposite direction of the z axis of the current frame
    if (qNorm<1e-4)
    {
        oneOverQuatErrRedNorm = 1e-4; // a small number.
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
    Eigen::Vector3d tauFF = parameters.inertiaMatrix*angVelDotEst_rps - (parameters.inertiaMatrix*angVel_rps).cross(angVel_rps);
    // eq. 21
    Eigen::Vector3d tauCtrl_Nm = parameters.attCtrlTiltPrio.KP * quatErrRed.vec() +
                                 parameters.attCtrlTiltPrio.KP(2,2) * mySignum(quatError.w()) * quatErrYaw.vec() +
                                 parameters.attCtrlTiltPrio.KD * angVelErr_rps +
                                 tauFF;
    return tauCtrl_Nm;
}

// Altitude Ref Dynamics

altCtrlRefStates Drone::altControlRefDyn(double zCmd, double timeStep_s)
{
    // Compute the control input (acceleration)
    double error = zCmd - g_altCtrlRefDynStates.posRef;
    double timeConst = parameters.altCtrlRefDyn.timeConst;
    double damping = parameters.altCtrlRefDyn.damping;
    double accNow = error *  timeConst * timeConst - 2.0 * damping * timeConst * g_altCtrlRefDynStates.velRef;
    // TODO: add acc limits
    g_altCtrlRefDynStates.accRef = accNow;
    // integrate to velocity now
    g_altCtrlRefDynStates.velRef = g_altCtrlRefDynStates.velRef + g_altCtrlRefDynStates.accRef * timeStep_s;
    //TODO: add vel limits
    // integrate to position now
    g_altCtrlRefDynStates.posRef = g_altCtrlRefDynStates.posRef + g_altCtrlRefDynStates.velRef*timeStep_s;

    return g_altCtrlRefDynStates;
}


//  Altitude PID control
altCtrlErrOutputs Drone::altPidControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, double timeStep_s)
{
    altCtrlErrOutputs outputs;
    // error
    double error = zDes_m - z_m;

    // d_error
    double d_error = dzDes_mps - dz_mps;

    // Proportional term
    double proportional = parameters.altCtrlPID.Kp * error;

    // Integral term
    g_altIntegral += parameters.altCtrlPID.Ki * error * timeStep_s;

    // todo: add proper anti-windup

    // Derivative term
    double derivative = parameters.altCtrlPID.Kd * d_error;

    // Calculate the thrust for height control
    // Following lines will implement the correct thrust computation (assumption: thrust aligned with the body z axis)
    double R33 = quat2R33(orientation);
    outputs.accCmd_mps2     =   (Environment::GRAVITY + proportional + g_altIntegral + derivative) / R33;
    outputs.controlThrust_N =   parameters.mass * outputs.accCmd_mps2;

    // Following lines implements generic PID, which will perform very well if you stick to the parametrization I gave in parameter.cpp.
    //double controlThrust_N =   proportional + g_altIntegral + derivative;

    return outputs;
}