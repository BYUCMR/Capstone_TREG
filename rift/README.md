# RIFT
This module handles the configuration, kinematics (forward and inverse), and
visualization of a rover made of inflatable, flexible tubes.

## arraytype.py
This file contains descriptions of important types for static type analysis.

## anim.py
This file contains functions used for visualizing the rover using plotly.
An example of its use can be found in `../tests/animate.py`.

## constrain.py
This file contains a system for describing the constraints we use to generate
motion commands for our robot.

## grav.py
This file contains a `Stabilizer` class that we use to simulate the effect of
gravity on the robot.

## robot.py
Here we have a class called `TrussRobot` that simulates a generic truss robot.
It has methods for updating its position based on commands and constraints
with either forward or inverse kinematics.

## rover.py
This is where we define the specific configuration of our physical robot. We
also define various kinds of motion that this robot can perform.

## steps.py
We use this file to help generate commands to control robots.

## tubetruss
This module contains classes and functions used to describe, simulate, and
control general variable-shape truss robots.
