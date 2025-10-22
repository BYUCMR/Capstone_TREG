import numpy as np

import path, truss_config
from motion import MotionPlanner
from truss_robot import TrussRobot
from viz import MotionFig, plot_theta_thetad

ol_config_3d = truss_config.CONFIG_3D_1
ol_config_2d = truss_config.CONFIG_2D_1

ol_path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))
ol_path_2d = path.make_path(dimension=2)

ol_robot = TrussRobot(ol_config_3d)

ol_planner = MotionPlanner(
    robot=ol_robot,
    path=ol_robot.move_node_pos + ol_path_3d,
)

ol_planner.move_ol()

t_hist = ol_robot.t_hist
theta_hist = ol_robot.roll_hist

plot_theta_thetad(ol_robot, save_fig=False, filename="theta_thetad_plot.png")


cl_config_3d = truss_config.CONFIG_3D_1
cl_path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))

cl_robot = TrussRobot(cl_config_3d)
cl_planner = MotionPlanner(
    robot=cl_robot,
    path=cl_robot.move_node_pos + cl_path_3d,
    figure=MotionFig(cl_robot),
)

for i, (t, theta) in enumerate(zip(t_hist, theta_hist)):
    if i==0:
        continue  # Skip the first entry since it's the initial condition
    theta = theta + np.random.normal(0, 0.01, size=theta.shape)  # Add small noise to simulate measurement error
    cl_planner.move_cl(theta, t, verbose_print_rate=10)
