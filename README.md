# YAAVS: Yet Another Autonomous Vehicles Simulator  

## Information

YAAVS simulates the movement of autonomous vehicles inside a mine.  
Robots objective is to go from their starting position to beacon area and then return to their start position.

This simulator is currently (2022) used in the *Autonomous and cooperative vehicular systems* course (DAT295/DIT669) at **Chalmers University of Technology**.
See the [project specifications](docs/project2022.md).

## Contents

* simulator -           The simulator source code
* Viewer -              The Visualizer source code
* logplayer -           The logplayer source code
* GUISample -           Graphical robot agent (C++) source code
* robsample -           robot agent (C) source code
* jClient -             robot agent (Java) source code
* pClient -             robot agent (Python) source code
* Labs -                examples of labyrinths used in previous competitions
* startAll -            script that runs the simulator, the visualizer and 5 GUISamples
* startSimViewer -      script that runs the simulator and the Viewer

## Install

The source code was compiled with gcc/g++ - Gnu Project C/C++ Compiler
(gcc version  9.3.0) using the Qt libraries (release 5.12.8) on Ubuntu 20.04.

It is required to have the development version of gcc/g++, cmake, Qt libraries
release 5.x installed in the system prior to compilation.
On Ubuntu 20.04 run the following:
```bash
sudo apt-get install build-essential cmake qt5-default qtmultimedia5-dev
```

Then in the repository base dir, execute:
```bash
cd build
cmake ..
make
```

To run the simulator, Viewer and C++ agent, execute (at the repository base dir):
```bash
./startAll
```

## Credits

YAAVS is based on [CiberRato](https://github.com/iris-ua/ciberRatoTools) Robot Simulation Environment by Universidade de Aveiro / IEETA.
