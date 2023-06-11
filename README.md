# droneSwarm

Small project for simulating multiple drones together.

Heavily WIP.

I have been developing another physics simulation [here](https://github.com/burakyueksel/physics) using C.
Let us see which one will prevail, or maybe they merge.

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
