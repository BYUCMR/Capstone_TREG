# RIFT
This module handles the configuration, kinematics (forward and inverse), and
visualization of a rover made of inflatable, flexible tubes.

## anim.py
This file contains functions used for visualizing the rover using plotly.
An example of its use can be found in `../tests/rover.py`.

## robot.py
We have two robot classes: one for forward kinematics, and one for inverse.

Both classes have configurable callbacks to respond to changes in state.

The inverse kinematics class is more fully developed and contains methods
for easily generating crawling motion.

## steps.py
This file contains functions to assist in building matrices describing desired
node motion.

## truss_config.py
This file contains code for generating possible starting configurations for
the rover.

## tubetruss.py
This file contains `TubeTruss`, which is a class that describes generalized
truss structures made from fixed-length tubes. The class has cached properties
for various matrices used for kinematic simulation.

## typing.py
This file contains typing constructs used as annotations elsewhere in the
module.
