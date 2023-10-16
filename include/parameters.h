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
