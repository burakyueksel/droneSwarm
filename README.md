# droneSwarm

Author: Burak Yüksel

Small project for simulating multiple drones together.

Heavily WIP.

Physics and everything that runs in compile time is written using C++.

Visualization part of the code is written using Python.

I have been developing another physics simulation [here](https://github.com/burakyueksel/physics) using C.
Let us see which one will prevail, or maybe they merge.

## What is in there already

- Create drone item in a swarm automatically from the amount of drones defined in parameters.cpp file.
- Second order rigid body dynamics evolving in SE(3) for each drone.
- For attitude representation quaternions are used, but conversion functions to rotation matrix SO(3) and Euler angles are added.
- Different drone types are added, but only multirotor basic dynamics are implemented.
- Altitude control with proper gain selection and assignment.
- Attitude control based on tilt prioritization with proper gain selection and assignment.
- Logging of drone states in the swarm.
- Visualisation of drone states in the swarm.
- Cmake test for CI/CD added.
- Altitude controller reference dynamics (2nd order) is added.
- Position controller reference dynamics (2nd order) is added.

## What is in pipleline

- Implement position controller for multirotors.
- Implement reference dynamics in the inner loops where it is missing.
- Add actuator models.
- Add actuator table per drone, add control effectiveness.
- Add control allocation.
- Improve visualization: animations/using UnrealEngine plug-in
- Integrate Acados for trying out optimal control solutions
- Add sensor models: IMU, Baro, GNSS, Radar, Lidar, Camera, OpticFlow, Airspeed, etc.
- Add dynamics for fixed-wing and VTOL types.
- Add control for fixed-wing and VTOL types too.
- Add communication models: radio, LTE. Drone2Drone and Drone2Ground
- Add swarm control algorithms (connectivity graphs, etc).
- Add RL and other ML methods for swarm control.
- Add more texture to the environment: e.g. 3D obstacles.

### Play around:

You can define your drone under src/parameters.cpp. Just copy-paste one of the existing ones, set the parameters the way you want it to behave, name the parameter ID correctly, and the rest should work out of box.

See below how to compile and run things.


### Compile

To run the code:

```console
cmake .
make
./build/DroneSwarmSimulation
```

### Analyse

You should see on your terminal some outputs. This also generates a log file in .txt format for later analysis.

Then you can run the analyze script from the same place in your folder tree

To run the code:

```console
python3 analyse/plotDrone.py
```

This will generate a plot similar to the following

![image](https://github.com/burakyueksel/droneSwarm/assets/40430575/3fcbd048-5241-40fe-8aaf-45f62853c9ef)

In this particular figure we are seeing that multiple drones are in freefall for 1 second.

After adding a tuned PID controller to the altitude, you will see something like this:

![image](https://github.com/burakyueksel/droneSwarm/assets/40430575/2deed7ed-7426-441c-a193-67de03369f71)


### Clean

A clean rule is implemented for your convenience. If you like to remove the binaries from the previous build (it will work only if you had a build before, meaning you have a generated Makefile), simply run

```console
make clean_files
```

## Shell scripts

Make sure you do

```console
chmod +x <name_of_the_sh_file.sh>
```
to the shell scripts.

Run them with

```console
./<name_of_the_sh_file.sh>
```



- runSimulation.sh will clean already existing Makefile, do cmake and make, run the simulator and plot the positions.
