import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

from rift.path import make_path
from rift.plotting import make_motion_fig
from rift.robot import RobotInverse
from rift.truss_config import CONFIG_2D_1 as config

path = make_path(dimension=2)
path += config.initial_pos[config.move_node]

robot = RobotInverse(config)
fig = make_motion_fig(robot, path)
for pos, vel in robot.move_node_along_path(config.move_node, path):
    fig.send((pos, vel))
