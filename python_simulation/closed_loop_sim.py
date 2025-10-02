import numpy as np

import truss_config
from truss_robot import Robot2D, Robot3D
from motion import MotionPlanner, MotionConstraintsGenerator
from viz import MotionViz, plot_theta_thetad

ol_config_3d = truss_config.CONFIG_3D_1
ol_config_2d = truss_config.CONFIG_2D_1

ol_robot = Robot3D(ol_config_3d, RPYrot=(90., -45.0, 45.0), path_scale=1, path_type="polygon", num_sides=4)
# ol_robot = Robot2D(ol_config_2d, RPYrot=[0], path_scale=1, path_type="polygon", num_sides=4)

ol_planner = MotionPlanner(
    robot=ol_robot,
    motion_constraints_generator=MotionConstraintsGenerator(ol_robot),
)

thetads, ol_robot = ol_planner.move_ol()

t_hist = ol_robot.t_hist
theta_hist = ol_robot.theta_hist

plot_theta_thetad(ol_robot, save_fig=False, filename="theta_thetad_plot.png")


cl_config_3d = truss_config.CONFIG_3D_1

cl_robot = Robot3D(cl_config_3d, RPYrot=(90., -45.0, 45.0), path_scale=1, path_type="polygon", num_sides=4)
cl_planner = MotionPlanner(
    robot=cl_robot,
    motion_viz=MotionViz(cl_robot),
    motion_constraints_generator=MotionConstraintsGenerator(cl_robot),
)

for i, (t, theta) in enumerate(zip(t_hist, theta_hist)):
    if i==0:
        continue  # Skip the first entry since it's the initial condition
    theta = theta + np.random.normal(0, 0.01, size=theta.shape)  # Add small noise to simulate measurement error
    thetad, cl_robot, finished = cl_planner.move_cl(t=t, dt=0.01, thetas=theta, verbose_print_rate=10)
    if finished:
        break
