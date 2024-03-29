/*
 * File: simulator.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "drone.h"
#include "environment.h"
#include "parameters.h"
#include <iostream>
#include <fstream>

int main()
{
    // Open a file to store the positions
    std::ofstream outputFile("drone_data.txt");
    // Load the parameters
    Parameters::loadParameters();

    std::vector<Drone> drones;
    for (int i = 0; i < Parameters::droneParams.size(); ++i)
    {
        drones.emplace_back(i);
    }

    int numSteps = Environment::timeEnd/Environment::timeStep;

    for (int step = 0; step < numSteps; ++step)
    {
        double currentTime = step * Environment::timeStep;
        std::cout << "Simulation Time: " << currentTime << " seconds" << std::endl;
        std::cout << "Number of Drones: " << drones.size() << std::endl;
        for (auto& drone : drones)
        {
            // get the translational states of each drone
            Eigen::Vector3d position = drone.getPosition();
            Eigen::Vector3d velocity = drone.getVelocity();
            Eigen::Quaterniond quaternion = drone.getQuaternion();
            Eigen::Vector3d angVel_prs = drone.getBodyRates();
            // x and y pos command for each drone in NED
            horizontalStates xyCmd_m;
            xyCmd_m.x = 0.0;
            xyCmd_m.y = 0.0;
            // z pos command for each drone in NED
            double zCmd_m = -50;
            /* POSITION CONTROLLER */
            posCtrlRefStates posRefStates = drone.posControlRefDyn(xyCmd_m, Environment::timeStep);
            horizontalStates posAccCmdXY  = drone.posCtrlErr(posRefStates, position, velocity, Environment::timeStep);
            /* ALTITUDE CONTROLLER */
            // reference dynamics
            altCtrlRefStates altRefStates = drone.altControlRefDyn(zCmd_m, Environment::timeStep);
            // error dynamics
            altCtrlErrOutputs altCtrlOutputs = drone.altPidControl(altRefStates.posRef, position.z(), altRefStates.velRef, velocity.z(), Environment::timeStep);
            /* ATTITUDE CONTROLLER */
            Eigen::Quaterniond quatDes = drone.attTiltPrioRefDyn(posAccCmdXY.x, posAccCmdXY.y, -altCtrlOutputs.accCmd_mps2, 0);
            // test direct attitude commands
            //Eigen::Quaterniond quatDes = drone.eulerToQuaternion(30, 30, 30);
            Eigen::Vector3d angVelDes_rps (0,0,0);
            Eigen::Vector3d angVelDotEst_rps (0,0,0);
            Eigen::Vector3d torqueCtrl = drone.attTiltPrioControl(quatDes, quaternion, angVelDes_rps, angVel_prs, angVelDotEst_rps);
            // set the external torques and forces
            drone.setExternalTorqueBody(torqueCtrl);
            drone.setExternalForceBody(Eigen::Vector3d(0.0, 0.0, altCtrlOutputs.controlThrust_N));
            // update all states
            drone.updateState(Environment::timeStep);
            // OUTPUT TO THE TERMINAL
            // Print the position of each drone
            //std::cout << xyCmd_m.x << "     " << xyCmd_m.y << std::endl;
            /*
            std::cout << "Drone " << drone.getID() << " position: "
                      << position.x() << ", "
                      << position.y() << ", "
                      << position.z() << std::endl;
            // Print the quaternion of each drone
            std::cout << "Drone " << drone.getID() << " unit quaternion: "
                      << drone.getQuaternion().w() << ", "
                      << drone.getQuaternion().x() << ", "
                      << drone.getQuaternion().y() << ", "
                      << drone.getQuaternion().z() << std::endl;
            */
            // OUTPUT TO THE FILE
                    // Store the positions in the file
            outputFile << currentTime
                       << " " << drone.getID()
                       << " " << position.x() << " " << position.y() << " " << position.z()
                       << " " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
                       << " " << posRefStates.posRef.x << " " << posRefStates.posRef.y << " " << altRefStates.posRef
                       << " " << posRefStates.velRef.x << " " << posRefStates.velRef.y << " " << altRefStates.velRef
                       << " " << posRefStates.accRef.x << " " << posRefStates.accRef.y << " " << altRefStates.accRef
                       << "\n";
        }
/*
        for (auto& drone : drones)
        {
        }
*/
        // Perform other simulation tasks
    }
    // Close the output file
    outputFile.close();

    return 0;
}
