# RIFT
This module handles the configuration, kinematics (forward and inverse), and
visualization of a rover made of inflatable, flexible tubes.

## anim.py
This file contains functions used for visualizing the rover using pyqtgraph.
An example of its use can be found in `../demos/animate.py`.

## arraytypes.py
This file contains descriptions of important types for static type analysis.

## grav.py
This file contains a `Stabilizer` class that we use to simulate the effect of
gravity on the robot.

## gui
The code in this module sets up a graphical user interface for controlling
a simulated or physical robot.

## motion
This module contains a generalized system for describing steps for robots to
take.

### constraints.py
This file contains a system for describing the constraints we use to generate
motion commands for our robot.

### optimize.py
This file contains functions that we use to select optimal motion.

### steps.py
This is where we bring the whole motion interface together.

## rover.py
This is where we define the specific configuration of our physical robot. We
also define various kinds of motion that this robot can perform.

## steps.py
We use this file to help generate commands to control robots.

## transmit
This module contains all the code we use to communicate with and control a
physical robot.

## tubetruss
This module contains classes and functions used to describe, simulate, and
control general variable-shape truss robots.

### linalg.py
This file contains supplementary linear algebra functions used elsewhere in
the module.

### robotics.py
This file contains the systems we use to simulate and control truss robots.

### singularity.py
This file contains a system for avoiding singular states.

The `TrussRobot` class within has methods for updating its position based on
commands and constraints with either forward or inverse kinematics.
