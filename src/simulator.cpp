/*
 * File: simulator.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "drone.h"
#include "parameters.h"
#include <iostream>
#include <fstream>

int main()
{
    // Open a file to store the positions
    std::ofstream outputFile("drone_positions.txt");
    // Load the parameters
    Parameters::loadParameters();

    std::vector<Drone> drones;
    for (int i = 0; i < Parameters::droneParams.size(); ++i)
    {
        drones.emplace_back(i);
    }

    double timeStep = 0.001;
    double timeEnd  = 1;
    int numSteps = timeEnd/timeStep;

    for (int step = 0; step < numSteps; ++step)
    {
        double currentTime = step * timeStep;
        std::cout << "Simulation Time: " << currentTime << " seconds" << std::endl;
        std::cout << "Number of Drones: " << drones.size() << std::endl;
        for (auto& drone : drones)
        {
            drone.updateState(timeStep);
            // OUTPUT TO THE TERMINAL
            // Print the position of each drone
            std::cout << "Drone " << drone.getID() << " position: "
                      << drone.getPosition().x() << ", "
                      << drone.getPosition().y() << ", "
                      << drone.getPosition().z() << std::endl;
            // Print the quaternion of each drone
            std::cout << "Drone " << drone.getID() << " unit quaternion: "
                      << drone.getQuaternion().w() << ", "
                      << drone.getQuaternion().x() << ", "
                      << drone.getQuaternion().y() << ", "
                      << drone.getQuaternion().z() << std::endl;
            // OUTPUT TO THE FILE
                    // Store the positions in the file
            Eigen::Vector3d position = drone.getPosition();
            outputFile << drone.getID() << " " << position.x() << " " << position.y() << " " << position.z() << "\n";
        }

        // Perform other simulation tasks

        for (auto& drone : drones)
        {
            drone.setExternalTorque(Eigen::Vector3d(0.0, 0.0, 0.0));
            drone.setExternalForce(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
        // Perform other simulation tasks
    }
    // Close the output file
    outputFile.close();

    return 0;
}
