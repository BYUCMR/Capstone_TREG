import numpy as np

import anim, plotting
from linalg import roll_pitch_yaw
from path import make_path
from robot import RobotForward, RobotInverse
from truss_config import CONFIG_3D_1 as config

path = make_path(xform=roll_pitch_yaw(np.pi/2, -np.pi/4, np.pi/4))
path += config.initial_pos[config.move_node]

ol_robot = RobotInverse(config)

for _ in ol_robot.move_node_along_path(config.move_node, path):
    pass

plotting.plot_theta_thetad(ol_robot, save_fig=False, filename="theta_thetad_plot.png")


cl_robot = RobotForward(config)
cl_plotter = anim.RobotPlotter3D(cl_robot)

animation = anim.animate_robot(cl_plotter, path)
for roll in ol_robot.roll_hist:
    roll += np.random.normal(0, 0.01, size=roll.shape)  # Add small noise to simulate measurement error
    cl_robot.update_state_from_roll(roll)
    vel = np.array([0.] * cl_robot.dim)
    animation.send((cl_robot.pos_of(config.move_node), vel))
