import numpy as np

import path, truss_config, viz
from motion import MotionPlanner
from truss_robot import TrussRobot
from viz import plot_theta_thetad

ol_config_3d = truss_config.CONFIG_3D_1
ol_config_2d = truss_config.CONFIG_2D_1

ol_path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))
ol_path_2d = path.make_path(dimension=2)

ol_robot = TrussRobot(ol_config_3d)

ol_planner = MotionPlanner(ol_robot)

for _ in ol_planner.move_node_along_path(
    ol_robot.config.move_node,
    ol_robot.move_node_pos + ol_path_3d,
):
    pass

t_hist = ol_robot.t_hist
theta_hist = ol_robot.roll_hist

plot_theta_thetad(ol_robot, save_fig=False, filename="theta_thetad_plot.png")


cl_config_3d = truss_config.CONFIG_3D_1
cl_path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))

cl_robot = TrussRobot(cl_config_3d)

fig = viz.make_motion_fig(cl_robot, cl_robot.move_node_pos + cl_path_3d)
next(fig)
for i, (t, theta) in enumerate(zip(t_hist, theta_hist)):
    if i==0:
        continue  # Skip the first entry since it's the initial condition
    theta = theta + np.random.normal(0, 0.01, size=theta.shape)  # Add small noise to simulate measurement error
    cl_robot.update_state_from_roll(theta, t)
    vel = np.array([0.] * cl_robot.dim)
    fig.send((cl_robot.move_node_pos, vel))
