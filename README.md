# Robot

Main idea of this repository is to provide classes to control robot made with BLDC LK-TECH drives with RS-485 data control. As a plus there is no  limit for joints number. 

Linear axes are also foreseen in concept (but not implemented yet).
Every motor contains static XYZ shifts to take into account mechanical design of robot and tool length.

Highlights:
* Any joints ammount
* LKMTECH python drivers for BLDC drives. MS9025 are tested. See https://aliexpress.com/store/912257172
* Tool tip coordinates calculation for robot with any number of motors
* XYZ coordinates to motor angles calculation. Means backward calculation problem is solved for N-joint machine
* TODO: G-codes interrpreter


![Prototype robot image](RobotScheme.svg?raw=true "Prototype")
