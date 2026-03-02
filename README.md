# 2025 BYU Capstone Team 9 (RIFT) Repository
This repository contains code and other resources pertaining to the RIFT team
from the BYU engineering department's 2025 Capstone program.

## MATLAB code
The `MATLAB_Simulation` directory contains the initial code from Dr. Usevitch
for simulating and visualizing a rover constructed from isoperimetric triangle
moduels. This simulation and visualization is run with the `Worm_Bot.m` script.

## Python code
### RIFT package
The RIFT Python package contains code for simulating and visualizing a rover
similar to the one simulated by the MATLAB code, but this one carries a
payload. It has also been simplified such that there is no offset between
connected triangle modules.

The package requires Python 3.13 or above. Its required libraries can be
installed via `pip install -r rift/requirements.txt`

### Tests
The RIFT package is used through various tests. The `animate.py` script runs
and displays a simulated rover walking. The `measure.py` script measures the
capabilities of a given rover configuration. The `prof.py` script profiles the
code used to simulate a walking rover.
