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

struct linSysParameters
{
    double timeConst;
    double damping;
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
    Eigen::Vector4d gains;
};

struct trajectory2Gains
{
    Eigen::Vector2d gains;
};

struct trajectory4States
{
    double position;
    double velocity;
    double acceleration;
    double jerk;
    double snap;
};

struct trajectory2States
{
    double position;
    double velocity;
    double acceleration;
};

struct trajectorySE3
{
    trajectory4States x;
    trajectory4States y;
    trajectory4States z;
    trajectory2States psi;
};


struct DroneParameters
{
    DroneTypes::Type droneType;
    double mass;
    Eigen::Matrix3d inertiaMatrix;
    Eigen::Vector3d cogOffset;
    Eigen::Vector3d initPos;
    linSysParameters posCtrlRefDyn;
    linSysParameters altCtrlRefDyn;
    pidParameters posCtrlPID;
    pidParameters altCtrlPID;
    attCtrlTiltPrioParameters attCtrlTiltPrio;
    trajectory4Gains traj4Gains;
    trajectory2Gains traj2Gains;
};

namespace Parameters {
    extern std::vector<DroneParameters> droneParams;
    void loadParameters();
}
