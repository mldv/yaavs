
# YAAVS description

YAAVS is the acronym of Yet Another Autonomous Vehicles Simulator.
This software is based on the [CiberRato](https://github.com/iris-ua/ciberRatoTools) Robot Simulation Environment by Universidade de Aveiro / IEETA.

## Introduction

The YAAVS system creates a virtual arena where vehicles operate. 
The arena represents a mine, and it is a 2D space populated by walls and special areas. 
The system also creates the virtual bodies of the mining vehicles (also called *robots*). 
Users must provide the software agents which control the movement of the vehicles, in order to accomplish some tasks.

All vehicles have the same kind of body, 
which is composed of a circular base, equipped with sensors and actuators. 

The simulator:
* sends sensor measures to the agents, 
* receives and applies actuating orders from the agents. 

Each vehicle has to be controlled by an independent program, i.e., the fleet is actually a collection of, possibly identical, programs. 
Vehicles are allowed to communicate with each other within given restrictions. 
Score depends on fulfilment of challenge goals and on penalties.

The task of the user is to develop and run the software that drives vehicles.


## Simulation environment

The YAAVS software is based on a distributed architecture, where three different type of applications enter into play: the simulator, the visualizer, and the agents.

![Overview of YAAVS software](img/software_components.png)

The **simulator** is responsible for:

- Implementing the virtual bodies of the robots.
- Estimating sensor measurements and sending them to the corresponding agent.
- Moving robots within the arena, according to orders received from corresponding agent and taking into account environment restrictions. For instance, a robot can not move over an obstacle.
- Routing the messages sent by robots, taking into account communication constraints.
- Updating robot score, taking into account the fulfilled goals and applied penalties.
- Sending scores and robots positions to the visualizer.
- Making available a control panel to start/restart and stop the competition.

The **visualizer** is responsible for:

- Graphically showing robots in competition arena, including their states and scores.
- Making available a control panel to start/restart and stop the competition.

The simulation system is discrete and time-driven. 
In each time step the simulator sends sensor measurements to agents, receives actuating orders, applies them, and updates scores. 
The cycle time `time_unit` is a parameter, with a default value of 50 ms.
All time intervals are measured as multiples of the cycle time.

All elements into play, namely arena, obstacles, and robots, are virtual, thus there is no need for a real length unit.
Hence, we use `d` as the unit of length, which corresponds to vehicles'
diameter.


## Arena

The arena represents a mine.

The arena is composed of a rectangular arena, outer delimited, with obstacles, target (*loading*) areas, a home (*unloading*) area and a starting grid inside.

Obstacles are fixed elements placed within the arena to limit the robot movements.
The target areas are circular with a beacon in its center.
The starting grid defines the robots initial positions.
The home area is a circle, without a beacon in its center, centered in grid position number 1.

Target areas are where mining vehicles collect gold (loading); the home area is where mining vehicles deliver gold (unloading).

![Example of an arena with five mining vehicles](img/arena_mine.png)

## Vehicles

All vehicles are two-wheeled, have a circular shape, and are equipped with sensors and actuators.
Moreover, vehicles can communicate.
Details about vehicle's equipment is given in what follows.

![Vehicle's body with sensors and actuators](img/robot_body.png)

### Sensors

Each vehicle is equipped with: 4 obstacle sensors, a collision sensor (a.k.a. *bumper*), a compass, a GPS, a ground sensor, and a beacon sensor.

Sensor models try to represent real devices.
Thus, on one side, their measures are noisy. 
On the other side, the reading of sensors is affected latency.
The amount of noise and latency is configurable for each sensor.
A latency of *N* means that the measurement available to the agent is *N* time units old.

A description for each kind of sensor follows.

#### Obstacle sensors 
They measure distances between the robot and its surrounding obstacles, including other robots. 
They can be put in any place in the robot periphery.  
Each sensor has a 60° aperture angle. 
The measure is inversely proportional to the lowest distance to the detected obstacles, and ranges between 0.0 e 100.0, with a resolution of 0.1. 

#### Bumper
It corresponds to a ring put around the robot.
It acts as a boolean variable enabled whenever there is a collision.
No noise is applied to this sensor.

#### Compass
It is positioned in the center of the robot and measures its angular position with respect to the virtual North. 
We assume the X-axis is facing the virtual north. 
Its measures range from −180 to +180 degrees, with a 1 degree resolution. 

#### GPS
It is a device that returns the position of the robot in the arena, with resolution 0.1. 
It is located at the robot center. 
When the simulation starts, the arena is randomly positioned in the world, being the origin coordinates (the left, bottom corner) assigned a pair of values in the range 0–1000. 

#### Ground sensor
It is a device that detects if the robot is completely on a target area or the home area. 
The ground sensor return an integer >=0, which corresponds to the target ID, or -1 when the robot is not completely on a target/home area.


#### Beacon sensor
It is positioned in the center of the robot, and has an omnidirectional covering.
It measures the angular position of the beacon with respect to the robot’s frontal axis.
The measure ranges from −180 to +180 degrees, with a resolution of 1 degree.
A beacon is not detected by the beacon sensor, if
- there is a high obstacle between it and the sensor (we say the robot is in a shadow area);
- the distance from it to the sensor is higher than 5 units of lenght (robot diameters).

### Actuators

The virtual robot has two motors, each driving a wheel. 

#### Motors
The motors try to represent, although roughly, real motors. 
Thus, they have inertia and noise. 
The 2 motors drive two wheels, placed as shown in the figure above.
Robot movement depends on the power applied to the two motors. 
Both translational and rotational movements are possible. 
If the same power values are applied to both motors the robot moves along its frontal axis. 
If the power values are symmetric the robot rotates.

The power accepted by motors ranges between −0.15 e +0.15, with resolution 0.001. 
However, this is not the power applied to wheels because of inertia and noise. 
See [*Movement model section* ](#movement-model)for a description of the input/output power relationship, that is, the relationship between power requested by agents and power applied to wheels. 

A power order applied to a motor keeps in effect until a new order is given.
In other words, if an agent applies a given power to a motor at a given time step, that power will be continuously applied in the following time steps until a new power order is sent by the agent.


### LEDs

Vehicles are equipped with three signalling LEDs.

The 3 LEDs are named:
- *visiting led*, 
- *returning led*, and
- *end led*.

The status of the LEDs can be set both by the agent or by the simulator, depending on the circumstances. 
They are used to notify the simulator or the agent of a change in the robot status (e.g., if it is loaded with gold). 
The way they must be used depends on the configuration.


### Communication between vehicles

Vehicles are equipped with a broadcast communication system. 

A vehicle can broadcast a message to all other vehicles. But there are constraints:
- Each vehicle can broadcast a single message (max 100 bytes) per time cycle, and receive up to one full message from each other vehicle. 
- A message is received by a vehicle only if the distance from the sender to the receiver is less than 8 distance units. Obstacles do not interfere with communication. 
- There is a latency of 1 cycle for every communicated message. 
- A fully linked configuration is not guaranteed at start. This means that, when the robots are in the starting grid positions, is not guaranteed that a message from a robot can be routed to all the others. 
- All communications between robots must be performed through the simulator by sending appropriate commands. Direct communication between agents, although technically possible, is considered cheating.


## Movement model

Vehicle’s position is given by `(x, y, θ)`, where `x` and `y` define the location of the center of the vehicle, and `θ` specifies its orientation. 

The following algorithm determines the new position of the vehicle at the following time step, given the input `(in_left, in_right)` that corresponds to the command sent by the agent to the simulator for wheels' motors.

The effective power that is applied to the motor takes into account inertia and noise, and it is derived as follows for the left wheel:
```text
noise = Norm(1, σ²)
speed_left = (in_left + prev_speed_left) * noise / 2
```
where noise is a random value generated form a Gaussian distribution with mean 1 and variance `σ²`, and  `prev_speed_left` is the previous value of `speed_left`.
The same is applied to the right wheel, independently.

Then, the movement is split in a translation of the vehicle position, considering its current
orientation, followed by a change of the orientation of the vehicle. 
For the translation one has
```text
lin = (speed_right + speed_left) / 2
x += lin * cos(θ)
y += lin * sin(θ)
```
and for the rotation
```text
θ += (speed_right - speed_left ) / d 
```
where `d` is the vehicle's diameter. 

Note that this provides the new vehicle position `(x , y , θ)` at the next step, **if no collisions occur**. 
In case of a collision, the simulator applies the rotational component only.


## Communication protocol

Communication between simulator and agents is based on UDP sockets, being the messages formatted into XML structures. 
There are five message tags to consider: 
- [request for registry](#request-for-registry), 
- [grant response](#grant-response), 
- [refusal response](#refusal-response), 
- [sensor data](#sensor-data), and 
- [actuation orders](#actuating-orders). 

Note that you need to implement the communication protocol only if you use a programming language different from C, C++, Java, or Python. Otherwise, you can use the available implementations (see example robots).

### Request for registry
The agent registers itself on the simulator sending a request for registry message to port 6000 of the IP address of the computer running the simulator. The message looks like
```xml
<Robot Name="name" Id="pos">
    <IRSensor Id="sid" Angle="sangle"/>
    .
    .
    .
</Robot>
```

where:
- `name` is the robot name, the one appearing in the scoreboard;
- `pos` is the robot position in the starting grid;
- `sid` is the id of an obstacle sensor, ranging from 0 to 3;
- `sangle` is the angular position of the sensor in robot periphery, ranging from -180.0 to +180.0.

Tags IRSensor are optional. You must use them if you want to change the default position of the obstacle sensors.

### Refusal response
If the simulator refuses the request for registry it sends to the agent the message
```xml
<Reply Status="Refused"></Reply>
```

### Grant response
If the simulator accepts the request for registry it sends to the agent the message
```xml
<Reply Status="Ok">
    <Parameters SimTime="time" KeyTime="time" CycleTime="time"
    NBeacons="nbeacons"
    BeaconNoise="noise" CompassNoise="noise"
    ObstacleNoise="noise" MotorsNoise="noise" />
</Reply>
```
where
- `time` is an integer value, representing a time, in unit-of-time;
- `nbeacons` is the number of beacons/target area;
- `noise` is a real value, representing a noise level.

The agent must memorize the port where this response came from and send all new messages to there.

### Sensor data
After registration, the simulator, at every cycle, sends to the robot a message with sensor data. 
The message sent is a subset of the one shown bellow, the subset depending on the sensor requests received.
```xml
<Measures Time="time">
    <Sensors Compass="angle" Collision="yesno" Ground="targetid">
        <BeaconSensor Id="0" Value="beaconmeasure"/>
        <IRSensor Id="0" Value="irmeasure"/>
        <IRSensor Id="1" Value="irmeasure"/>
        <IRSensor Id="2" Value="irmeasure"/>
        <IRSensor Id="3" Value="irmeasure"/>
        <GPS X="coord" Y="coord"/>
        <Message From="robotid"><![CDATA[msg]]></Message>
    </Sensors>
    <Leds EndLed="onoff" VisitingLed="onoff" ReturningLed="onoff"/>
    <Buttons Start="onoff" Stop="onoff"/>
</Measures>
```
where
- `time` is an integer value representing a time;
- `angle` is a real value representing an angle, in radians;
- `yesno` is the word "Yes" or "No";
- `targetid` is an integer: 0 if the robot is completely inside the home area, >0 if it is completely inside a target area (the value represents the ID of the target area) and −1 otherwise;
- `beaconmeasure` is a real value representing a beacon sensor measure, if beacon is visible, or the word "NotVisible" otherwise;
- `irmeasure` is a real number, representing an obstacle sensor measure;
- `coord` is a real number, representing a GPS spatial position;
- `msg` is the message broadcast by robot number robotid;
- `onoff` is the word "On" or "Off", representing a LED or button state.

### Actuating orders
At each cycle, agents can send to the simulator one or more actuating orders. 
However, the number of orders per device is limited to one. If more than one is received, only the last one will be considered. 
Each actuating order message is a subset of
```xml
<Actions LeftMotor="pow" RightMotor="pow" EndLed="act">
    <SensorRequests IRSensor0="Yes" IRSensor1="Yes"
    Beacon0="Yes"
    Ground="Yes" Compass="Yes"/>
    <Say><![CDATA[msg]]></Say>
</Actions>
```
where
- `pow` is a real value, representing a power;
- `act` is the word "On" or "Off", representing an order to turn-on or turn-off a LED;
- `msg` is the message to be broadcast; 

The number of sensor requests per cycle can be limited in number (is a configuration setting), if more are requested, an arbitrary subset with the maximum dimension allowed is considered.
Motor orders are persistent, in the sense that the order is kept until a new one is received by the simulator.
Sensor requests are not persistent. 
If an agent wants to read the same sensors in two or more consecutive cycles, it needs to send the sensor requests on each cycle.


## Configuration and default values

The configuration of the simulation is specified by the *scoring system* and three XML files: the parameter, the arena, and the grid.

The scoring system needs to be specified as an integer of the `--scoring` argument of the simulator program.
The last version of the scoring system is v7: `--scoring 7`, which is also the default value.

The XML files are self-explanatory, in what follows the default parameters.


Configuration parameters:
```xml
<Parameters 
        SimTime="5000"
        CycleTime="25"
        CompassNoise="0" BeaconNoise="0" ObstacleNoise="0.1"
        MotorsNoise="0" KeyTime="5000"
        GPS="On" GPSLinNoise="0.0" GPSDirNoise="0"
        BeaconSensor="On"
        CompassSensor="On"
        ScoreSensor="On" ShowActions="True" NBeacons="2"
        NRequestsPerCycle="0"
        ObstacleRequestable="Off" BeaconRequestable="Off"
        GroundRequestable="Off" CompassRequestable="Off"
        CollisionRequestable="Off"
        ObstacleLatency="0" BeaconLatency="0"
        GroundLatency="0" CompassLatency="0"
        CollisionLatency="0"
        BeaconAperture="3.141593"
        ReturnTimePenalty="25" ArrivalTimePenalty="100"
        CollisionWallPenalty="2" CollisionRobotPenalty="1"
        TargetReward="100" HomeReward="100"
        Lab="../Labs/chalmersDAT295-22/C1-lab.xml"
        Grid="../Labs/chalmersDAT295-22/C1-grid.xml"
/>
```

Arena:
```xml
<Lab Name="DAT295-22-1" Height="14" Width="28">
    <Target X="3" Y="11.0" Radius="3.0"/>
    <Beacon X="17" Y="11" Height="1.0"/>
    <Target X="17" Y="11" Radius="1.0"/>
    <Beacon X="9" Y="5.0" Height="1.0"/>
    <Target X="9" Y="5.0" Radius="1.0"/>
    <Wall Height="2.5">
        <Corner X="24.5" Y="10.5"/>
        <Corner X="26.0" Y="9.0"/>
        <Corner X="27.5" Y="6.0"/>
        <Corner X="27.5" Y="3.0"/>
        <Corner X="28.0" Y="3.0"/>
        <Corner X="28.0" Y="14.0"/>
        <Corner X="25.5" Y="14.0"/>
        <Corner X="24.0" Y="11.0"/>
    </Wall>
    <Row Pos="12" Pattern="           +              |              "  />
    <Row Pos="11" Pattern="  +  +  +  +  +  +  +  +  +  +  +  +  +  "  />
    <Row Pos="10" Pattern="                          |              "  />
    <Row Pos="9"  Pattern="  +--+--+--+--+--+--+--+  +--+  +  +  +  "  />
    <Row Pos="8"  Pattern="     |           |     |     |           "  />
    <Row Pos="7"  Pattern="--+  +  +  +  +--+  +  +--+  +--+  +  +  "  />
    <Row Pos="6"  Pattern="  |  |        |     |  |  |     |        "  />
    <Row Pos="5"  Pattern="--+  +--+--+--+  +--+  +--+--+  +  +  +  "  />
    <Row Pos="4"  Pattern="     |           |  |  |        |        "  />
    <Row Pos="3"  Pattern="  +--+  +--+--+--+  +  +  +--+--+  +  +  "  />
    <Row Pos="2"  Pattern="  |  |  |           |     |              "  />
    <Row Pos="1"  Pattern="  +--+  +  +  +  +  +  +  +  +  +  +  +  "  />
    <Row Pos="0"  Pattern="        |                                "  />
</Lab>


```

Starting grid:
```xml
<Grid>
    <Position X="3.0" Y="11.0"  Dir="0.0"/>
    <Position X="5.0" Y="11.0"  Dir="0.0"/>
    <Position X="7.0" Y="11.0"  Dir="0.0"/>
    <Position X="3.0" Y="13.0"  Dir="0.0"/>
    <Position X="5.0" Y="13.0"  Dir="0.0"/>
</Grid>
```
