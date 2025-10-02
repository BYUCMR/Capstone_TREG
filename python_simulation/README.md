# Isoperimetric Kinematics Truss Robot Project

This repository contains code for simulating, optimizing, and visualizing the motion of a truss robot made of inflatable tubes and rollers. The robot's geometry, constraints, and actuation are modeled using graph theory and optimization techniques. The code is modular, with clear separation between robot modeling, motion planning, constraints, and visualization.

## Demo Video

See the results and main demo in the following video:

**[Demo Video - Results](results/plot_demo_Sep25_2025.mp4)**

## File Overview

### motion.py
**Purpose:** Main entry point for running a motion planning demo. Sets up the robot, constraints, and runs the optimization loop. Contains:
- `MotionParams`: Data class for simulation parameters.
- `MotionConstraintsGenerator`: Generates and updates constraint matrices for optimization (move, lock, loop, broken roller constraints).
- `MotionPlanner`: Runs the optimization, updates robot state, and manages the simulation loop.
- Main block: Instantiates a robot, planner, and runs the demo.

### truss_robot.py
**Purpose:** Defines the truss robot's geometry, kinematics, and update logic. Contains:
- `TrussRobot` (abstract base class): Defines common methods for truss robots, including rigidity matrix calculation, edge/triangle setup, and update routines.
- `Robot2D`: 2D truss robot implementation (inherits from `TrussRobot`). Handles 2D geometry, plotting, and updates.
- `Robot3D`: 3D truss robot implementation (inherits from `TrussRobot`). Handles 3D geometry, plotting, and updates.

### path.py
**Purpose:** Generates geometric paths for the robot to follow (line, circle, polygon, etc.). Contains:
- `transform_path`: Function for generating and transforming paths based on type, dimension, and rotation.

### common_data.py
**Purpose:** Stores geometric and connectivity data for different robot configurations. Contains:
- `Data3D`: 3D robot configuration data (supports, edges, triangles, initial positions). Each configuration is accessed by an index.
  - Index 1 corresponds to an octahedron robot.
  - Index 2 corresponds to another octahderon robot, but the triangles are composed of different node arrangements that index 1.
- `Data2D`: 2D robot configuration data. Each configuration is accessed by an index.
  - Index 1 corresponds to a single triangle.
  - Index 2 corresponds to 3 triangles.

### viz.py
**Purpose:** Visualization utilities for robot motion and history plots. Contains:
- `MotionViz`: Interactive visualization of robot motion, theta, and thetad histories with control panel for toggling traces.
- `PlotViz`: Utility for static plotting and saving theta/thetad plots.

### closed_loop_sim.py
**Purpose:** Testing the ability of the robot to estimate its state given noisy position feedback data.

## How to Run the Main Demo

To install the dependencies for the project, run:

```bash
pip install -r requirements.txt
```

To run the main demo, execute `motion.py`:

```bash
python motion.py
```

This will:
- Instantiate a truss robot (2D or 3D, depending on configuration).
- Set up motion planning and constraints (move, lock, loop, broken roller).
- Run the optimization loop to move the robot along a path.
- Visualize the robot and plot the history of joint angles (`theta`) and their rates (`thetad`).
- Save results and figures as needed.

## Key Terminology

- **Truss Robot:** A robot made of interconnected tubes (edges) and nodes, forming triangles. Actuated by rollers that redistribute tubing material.
- **Move Node:** The node in the truss robot that is actively moved toward a target position during the demo.
- **Support Nodes:** Nodes that are constrained to be fixed or allowed only to slide (floor constraints).
- **Edge:** A tube segment between two nodes.
- **Triangle:** Three edges forming a closed loop; the perimeter is often constrained to be constant (loop constraint).
- **Roller:** An actuator that changes the distribution of tubing material across a triangle's three sides.
- **Broken Roller:** A roller that is disabled (its rate of change, thetadot, is set to zero via a constraint).
- **Rigidity Matrix (R):** Matrix mapping node velocities (`xdot`) to edge length rates (`Ldot`).
- **Ldot:** Time rate of change of edge lengths.
- **xdot:** Time rate of change of node positions.
- **Bt:** Matrix mapping roller rates (`thetadot`) to edge length rates (`Ldot`).
- **L2th:** Pseudoinverse of Bt, mapping edge length rates to roller rates.
- **Loop Constraint:** Keeps the perimeter of each triangle constant during motion.

## Class Overview

### motion.py
- `MotionParams`: Holds simulation parameters (objective, broken rollers, time step, etc.).
- `MotionConstraintsGenerator`: Builds and updates constraint matrices for optimization.
- `MotionPlanner`: Runs the optimization loop, updates robot state, and manages visualization.

### truss_robot.py
- `TrussRobot`: Abstract base class for truss robots, with methods for geometry, kinematics, and updates.
- `Robot2D`: 2D truss robot implementation.
- `Robot3D`: 3D truss robot implementation.

### path.py
- `Path`: Generates and transforms geometric paths for robot motion.

### common_data.py
- `Data3D`: 3D robot configuration data.
- `Data2D`: 2D robot configuration data.

### viz.py
- `MotionViz`: Interactive visualization of robot motion and history.
- `PlotViz`: Static plotting and saving of theta/thetad histories.

## Results

See the demo video in the `results` folder for a visual summary of the robot's motion and optimization results.

## Contact

For questions or contributions, please contact the repo owners.

**Updated September 25, 2025 -- James Wade**
