/*
 * File: main.cpp
 * Author: Burak Yueksel
 * Date: 2023-06-01
 */
#include "simulator.h"

int main()
{
    int numDrones = 2;
    double simulationTime = 10.0;
    double timeStep = 0.01;

    Simulator simulator(numDrones);
    simulator.runSimulation(simulationTime, timeStep);

    return 0;
}
