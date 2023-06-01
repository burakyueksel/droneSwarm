#include "parameters.h"

namespace Parameters {
    std::vector<DroneParameters> droneParams;

    void loadParameters() {
        // Load the drone parameters from a file or any other source
        // Here, we manually define the parameters for 4 drones as an example

        // Drone 1 parameters
        DroneParameters drone1Params;
        drone1Params.mass = 1.0; // Set the mass for drone 1
        drone1Params.inertiaMatrix << 1.0, 0.0, 0.0,
                                      0.0, 2.0, 0.0,
                                      0.0, 0.0, 3.0;
        droneParams.push_back(drone1Params);

        // Drone 2 parameters
        DroneParameters drone2Params;
        drone1Params.mass = 1.5; // Set the mass for drone 1
        drone2Params.inertiaMatrix << 4.0, 0.0, 0.0,
                                      0.0, 5.0, 0.0,
                                      0.0, 0.0, 6.0;
        droneParams.push_back(drone2Params);

        // Drone 3 parameters
        DroneParameters drone3Params;
        drone1Params.mass = 2.0; // Set the mass for drone 1
        drone3Params.inertiaMatrix << 7.0, 0.0, 0.0,
                                      0.0, 8.0, 0.0,
                                      0.0, 0.0, 9.0;
        droneParams.push_back(drone3Params);

        // Drone 4 parameters
        DroneParameters drone4Params;
        drone1Params.mass = 2.5; // Set the mass for drone 1
        drone4Params.inertiaMatrix << 10.0, 0.0, 0.0,
                                      0.0, 11.0, 0.0,
                                      0.0, 0.0, 12.0;
        droneParams.push_back(drone4Params);
    }
}