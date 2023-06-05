# droneSwarm

Small project for simulating multiple drones together.

Heavily WIP.

I have been developing another physics simulation [here](https://github.com/burakyueksel/physics) using C.
Let us see which one will prevail, or maybe they merge.

To run the code:

```console
cmake .
make
./build/DroneSwarmSimulation
```

You should see on your terminal some outputs. This also generates a log file in .txt format for later analysis.

Then you can run the analyze script from the same place in your folder tree

To run the code:

```console
python3 analyse/plotDrone.py
```

A clean rule is implemented for your convenience. If you like to remove the binaries from the previous build (it will work only if you had a build before, meaning you have a generated Makefile), simply run

```console
make clean_files
```