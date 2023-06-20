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
            // altitude control thrust for each drone
            double zDes_m = -50;
            double dzDes_mps = 0;
            double thrustCtrl = drone.altPidControl(zDes_m, position.z(), dzDes_mps, velocity.z(), Environment::timeStep);
            // set the external torques and forces
            drone.setExternalTorque(Eigen::Vector3d(0.0, 0.0, 0.0));
            drone.setExternalForce(Eigen::Vector3d(0.0, 0.0, thrustCtrl));
            // update all states
            drone.updateState(Environment::timeStep);
            // OUTPUT TO THE TERMINAL
            // Print the position of each drone
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
