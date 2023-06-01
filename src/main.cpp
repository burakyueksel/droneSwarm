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
