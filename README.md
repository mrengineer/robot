# Robot

Main idea of this repository is to provide classes to control robot made with BLDC LK-TECH drives with RS-485 data control. As a plus there is not limit for joints number. 

Linear axes are also foreseen in concept (but not implemented yet).
Every motor contains static XYZ shifts to take into account mechanical design of robot and tool length.

Highlights:
* LKTECH python drivers for BLDC drives
* Tool tip coordinates calculation for robot with any number of motors
* XYZ coordinates to motor angles calculation. Means backward calculation problem is solved for N-joint machine
* TODO: G-codes interrpreter


![Prototype robot image](RobotScheme.svg?raw=true "Prototype")
