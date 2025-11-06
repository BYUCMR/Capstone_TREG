import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import numpy as np

from rift.anim import RoverPlotter3D, animate_robot
from rift.linalg import roll_pitch_yaw
from rift.path import make_path
from rift.robot import RobotInverse
from rift.truss_config import CONFIG_3D_ROVER1 as config

path = make_path(xform=roll_pitch_yaw(np.pi/4, np.pi/6, np.pi/4))
path += config.initial_pos[config.move_node]

robot = RobotInverse(config)
plotter = RoverPlotter3D(robot)
animation = animate_robot(plotter, path)
for pos, vel in robot.move_node_along_path(config.move_node, path):
    animation.send((pos, vel))
