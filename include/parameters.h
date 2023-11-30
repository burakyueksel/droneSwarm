/*
 * File: parameters.h
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */

#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations
#include <vector>

struct DroneTypes
{
    enum Type
    {
        MC_QUAD,
        MC_HEXA,
        MC_OCTO,
        FW_MONO,
    };
};

struct altCtrlErrOutputs
{
    double controlThrust_N;
    double accCmd_mps2;
};

struct pidParameters
{
    double Kp;
    double Ki;
    double Kd;
};

struct attCtrlTiltPrioParameters
{
    Eigen::Matrix3d KP;
    Eigen::Matrix3d KD;

};

struct pdParameters
{
    double Kp;
    double Kd;
};

struct horizontalStates
{
    double x;
    double y;
};

struct posCtrlRefStates
{
    horizontalStates posRef;
    horizontalStates velRef;
    horizontalStates accRef;
};

struct altCtrlRefStates
{
    double posRef;
    double velRef;
    double accRef;
};

struct trajectory4Gains
{
    Eigen::RowVector4d gains;
};

struct trajectory2Gains
{
    Eigen::RowVector2d gains;
};

struct flatOutputsSE3
{
    double x;
    double y;
    double z;
    double psi;
};

struct  trajectorySE3
{
    Eigen::Matrix<double, 5, 1> x; // 5x1: pos, vel, acc, jerk, snap
    Eigen::Matrix<double, 5, 1> y; // 5x1: pos, vel, acc, jerk, snap
    Eigen::Matrix<double, 5, 1> z; // 5x1: pos, vel, acc, jerk, snap
    Eigen::Matrix<double, 3, 1> psi; // 3x1: pos, vel, acc
};

struct dTrajectorySE3
{
    Eigen::Matrix<double, 4, 1> x; // 4x1: vel, acc, jerk, snap
    Eigen::Matrix<double, 4, 1> y; // 4x1: vel, acc, jerk, snap
    Eigen::Matrix<double, 4, 1> z; // 4x1: vel, acc, jerk, snap
    Eigen::Matrix<double, 2, 1> psi; // 2x1: vel, acc
};


struct DroneParameters
{
    DroneTypes::Type droneType;
    double mass;                    // mass, [kg], 1x1
    Eigen::Matrix3d inertiaMatrix;  // moment of inertia around center of gravity [kgmm^2], 3x3
    double actBW_rps;               // actuator bandwidth, [rad/s], 1x1
    Eigen::Vector3d cogOffset;      // offset of center of gravity w.r.t. center of actuation, in cog frame, [m], 3x1
    Eigen::Vector3d initPos;        // initial position, [m], 3x1
    pdParameters posCtrlRefDyn;     // first or second order linear system parameters
    pdParameters altCtrlRefDyn;     // first or second order linear system parameters
    pidParameters posCtrlPID;       // pid gains
    pidParameters altCtrlPID;       // pid gains
    attCtrlTiltPrioParameters attCtrlTiltPrio; // att ctrl PD gains
    trajectory4Gains traj4Gains;    // 4th order trajectory generator gains
    trajectory2Gains traj2Gains;    // 2nd order trajectory generator gains 
};

namespace Parameters {
    extern std::vector<DroneParameters> droneParams;
    void loadParameters();
}
