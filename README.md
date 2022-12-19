# SumoRobot
This repository contains the code used for our Sumo Robot during our participation in the RBTX 2022 Competition

# Hardware
For our robot, we have used various electronics and hardware including:

* 1 x Arduino Uno
* 1 x Arduino Uno Shield
* 2 x QTR-1A Reflectance Sensor
* 1 X Sabertooth 2 x 25A Motor Driver
* 5 x Digital Infrared Sensor
* 2 x A58SW31ZY Motors
* 1 x Bluetooth Module HC-05

# Procedure and how it works

## Setup
The setup section will first begin Serial Communication between the connection to our Serial Monitor as well as with our Bluetooth Module and with our Sabertooth Motor Driver. 

It then initializes all needed initializations such as input / output pins and QTR sensors. It will then await for a bluetooth signal to specify which battle mode it should be set up. Once that is received, it wll await for the signal to start running. After all input is given, it will delay for 4.85 seconds roughly before it starts running.

## Runtime
Based on the previous battle mode we have set up. It will then either run StrategyOne, StrategyTwo, StrategyThree, or StrategyFour.

If during runtime it's given the command to power off, it will not run the selected strategy and remain stationary.
