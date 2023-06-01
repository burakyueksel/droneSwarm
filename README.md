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

A clean rule is implemented for your convenience. If you like to remove the binaries from the previous build (it will work only if you had a build before, meaning you have a generated Makefile), simply run

```console
make clean_files
```