/*
 * File: simulator.h
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#pragma once

#include <vector>
#include "drone.h"

class Simulator
{
public:
    Simulator(int numDrones); // Constructor
    void runSimulation(double simulationTime, double timeStep); // Run the drone swarm simulation
    // Add more member functions as needed

private:
    std::vector<Drone> drones; // Vector to store the drones
    // Add other relevant private member variables
};