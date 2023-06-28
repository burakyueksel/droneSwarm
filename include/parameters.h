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

struct altCtrlPidParameters
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

struct posCtrlStates
{
    Eigen::Vector2d posRef;
    Eigen::Vector2d velRef;
    Eigen::Vector2d accRef;
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
    altCtrlPidParameters altCtrlPID;
    attCtrlTiltPrioParameters attCtrlTiltPrio;
};

namespace Parameters {
    extern std::vector<DroneParameters> droneParams;
    void loadParameters();
}
