### Code for ASV (Version 0.1)

We are developing the autonomous surface vehicle, including the environmental perception, behavioral planning, motion planning, path following, feedback control, etc. 

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

#### Introduction

* C++ 17
* communication: socket TCP/IP, serial communication, checksum, etc
* controller: PID controller, thrust allocation, actuator, etc
* math: library involving linear algebra, numerical analysis, etc
* fileIO: csv parser, Database (SqLite3), JSON, etc
* remotecontrol: remote controller
* sensors: GPS, IMU, Wind, etc



#### TODO: 

1. Hybrid A star
2. Perception (e.g. Lidar, Marine Radar, camera, etc)
3. Hardware-in-loop simulation
4. Finite state machine
5. Message broker
