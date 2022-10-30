# Projects for DAT295 @ Chalmers 2022

Project specifications for *Autonomous and cooperative vehicular systems* course (DAT295/DIT669) at Chalmers University of Technology.

Supervisor: Dr. Marco Della Vedova <marco.dellavedova@chalmers.se>.

## [proj-MDV1] Coordination of autonomous mining vehicles in a simulation environment

### Background
The mining industry is currently fostering investments in robots and artificial intelligence to solve safety issues and boost efficiency, with the potential to generate billions of dollars in savings.
Robotic machines are becoming increasingly widespread in modern mines, with the tendency toward virtualization, underground vision, and remote control.
Planning the coordinated movements of a complete fleet of vehicles is one of the most essential tasks involved in mining operations, involving issues such as decision-making, path planning, and scheduling.

### Goals
In this project, you have to program the AI of a fleet of autonomous mining vehicles within a simulation environment.
There are many challenges to be faced:
- controlling the movement of the vehicles;
- exploring the mine;
- locating the gold;
- planning the path from the gold location to the unloading area;
- coordinating the vehicles to transport the gold to the unloading area.

Project's participants can be divided into teams, each developing a own strategy, and the teams can compete in a final contest.
The overall goal is to transport as much gold as possible to the unloading area in a given amount of time.

### Simulation environment
The YAAVS simulation environment will be used.
It is a modified version of CiberRatoTools by University of Aveiro (Portugal) and it is written in C++.
The mine is modeled as a 2D space with walls that can be seen as a bi-dimensional array of fixed-size cells. Each cell is a square with side length equal to twice the diameter of the vehicle.
Each mining vehicle is equipped with two motors, each driving a wheel, and some sensors: a compass, four obstacle sensors, a beacon sensor, a ground sensor, and a collision sensor. Moreover, vehicles can communicate.

### Requirements
You have to be comfortable programming.
The code of this project can be written in a programming language of your choice, since the communication between your agents and the simulator is via socket.
There are sample agents to start with, that are written in C, C++, Java, and Python.


## Details

The YAAVS software has many configuration parameters, see the [documentation](yaavs_intro.md) for more details.

The exact configuration (i.e., the noises characterization, the availability of sensors, the preliminary knowledge of the arena, etc.) can be discussed in the project proposal.

The overall goal is to develop the AI of autonomous vehicles, such that they move as much gold as possible from the target positions to the unloading area in different (possible unknown) mines.
More specific and detailed tasks have to be defined in the project proposal.
