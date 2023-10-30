# Projects for DAT295 @ Chalmers 2023

Project specifications for *Autonomous and cooperative vehicular systems* course (DAT295/DIT669) at Chalmers University of Technology.

Supervisor: Marco Della Vedova <marco.dellavedova@chalmers.se>.

## [proj-MDV1] Coordination of autonomous mining vehicles in a simulation environment

### Background
The mining industry is currently fostering investments in robots and artificial intelligence to solve safety issues and boost efficiency, with the potential to generate billions of dollars in savings. Robotic machines are becoming increasingly widespread in modern mines, with the tendency toward virtualization, underground vision, and remote control. Planning the coordinated movements of a complete fleet of vehicles is one of the most essential tasks involved in mining operations, involving issues such as decision-making, path planning, and scheduling.

### Goals
In this project, you have to program the AI of a fleet of autonomous mining vehicles within the YAAVS simulation environment. 
There are many challenges to be faced, including:
- controlling the movement of the vehicles;
- exploring the mine;
- locating the gold;
- planning the path from the gold location to the unloading area;
- coordinating the vehicles to transport the gold to the unloading area.

The project can address some of or all these challenges: the configuration of the environment (e.g., the number of vehicles, the availability of the sensors, their noise, the dimension of the mine, the obstacles, ...), the tasks of the AI, and the specific objectives of the project shall be agreed between the students and the supervisor.

#### Example of project goals

**Focus on localisation**. 
The map of the mine is known, the vehicles do **not** know their position in the map, they do not have access to the GPS sensor, and they have to localize themselves *first*.

**Focus on coordination**.
The map of the mine is known, the vehicles know their position in the map, they have access to the GPS sensor, and they have to coordinate themselves to transport the gold to the unloading area.

**Focus on exploration and mapping**.
The map of the mine is unknown, the vehicles have access to the GPS sensor, and they have to explore the mine to find the gold.


### Requirements
You have to be comfortable with programming.
The code of this project can be written in a programming language of your choice, since the communication between your agents and the simulator is via socket.
There are sample agents to start with, that are written in C, C++, Java, and Python.


## Details

The overall goal is to develop the AI of autonomous vehicles, such that they move as much gold as possible from the target positions to the unloading area in different (possible unknown) mines.
More specific and detailed tasks have to be defined in the project proposal.

The YAAVS software has many configuration parameters, see the [documentation](yaavs_intro.md) for more details.
The exact configuration (i.e., the noises characterization, the availability of sensors, the preliminary knowledge of the arena, etc.) can be discussed in the project proposal.

Suggested configuration for this project:

- The first target area defined in the lab is the home area (for unloading). There is no beacon inside the home area. It is circular and centered in the first grid position.
- The other target areas are loading areas (with gold). They are circular with radius 1. There is a beacon inside, which is detectable from a distance equal to 1. Note that a vehicle can detect the beacon when its center is over the target area (no need for the robot th be entirely inside the target area).
- The ID of the home area is 0, while the ID of the loading areas are >0. The ground sensor returns this ID (or -1 if the robot is not on a target area), so it can be used by vehicles to understand on which type of ground they are.
- Target areas do not overlap.
- The loading status of vehicles is set by the simulator using the *returning LED*: the LED is ON when the vehicle is loaded. The LED is turned on/off automatically by the simulator as soon as a vehicle is: (i) unloaded on a loading area (the LED is turned on), (ii) loaded on the unloading area (the LED is turned off).
- The scoring system is the n.7 (the `--scoring 7` for the simulator), which is currently the default value.