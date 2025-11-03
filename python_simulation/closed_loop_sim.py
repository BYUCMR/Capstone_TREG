import numpy as np

import plotting
from path import make_path
from robot import Robot
from truss_config import CONFIG_3D_1 as config

path = make_path(RPYrot=(90., -45.0, 45.0))
path += config.initial_pos[config.move_node]

ol_robot = Robot(config)

for _ in ol_robot.move_node_along_path(config.move_node, path):
    pass

t_hist = ol_robot.t_hist
theta_hist = ol_robot.roll_hist

plotting.plot_theta_thetad(ol_robot, save_fig=False, filename="theta_thetad_plot.png")


cl_robot = Robot(config)

fig = plotting.make_motion_fig(cl_robot, path)
for t, theta in zip(t_hist, theta_hist):
    theta = theta + np.random.normal(0, 0.01, size=theta.shape)  # Add small noise to simulate measurement error
    cl_robot.update_state_from_roll(theta, t)
    vel = np.array([0.] * cl_robot.dim)
    fig.send((cl_robot.pos_of(config.move_node), vel))
