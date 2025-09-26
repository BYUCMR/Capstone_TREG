import numpy as np
from truss_robot import Robot2D, Robot3D
from motion import MotionPlanner, MotionParams
from viz import PlotViz

ol_robot = Robot3D(1, RPYrot=[90., -45.0, 45.0], path_scale=1, path_type="polygon", num_sides=4)
  # ol_robot = Robot2D(1, RPYrot=[0], path_scale=1, path_type="polygon", num_sides=4)

motion_params = MotionParams(animate_robot=False, refresh_rate=10,
                              verbose_print=False, objective="Ldot",
                              broken_rollers=None, dt=0.01)

ol_planner = MotionPlanner(robot=ol_robot, motion_params=motion_params)

thetads, ol_robot = ol_planner.move_ol(motion_params=motion_params)

t_hist = ol_robot.t_hist
theta_hist = ol_robot.theta_hist

plot_viz = PlotViz(ol_robot)
plot_viz.plot_theta_thetad(save_fig=False, filename="theta_thetad_plot.png")

motion_params = MotionParams(animate_robot=True, refresh_rate=10,
                              verbose_print=True, objective="Ldot",
                              broken_rollers=None, dt=0.01)

cl_robot = Robot3D(1, RPYrot=[90., -45.0, 45.0], path_scale=1, path_type="polygon", num_sides=4)
cl_planner = MotionPlanner(robot=cl_robot, motion_params=motion_params)

for i, (t, theta) in enumerate(zip(t_hist, theta_hist)):
    if i==0:
        continue  # Skip the first entry since it's the initial condition
    theta = theta + np.random.normal(0, 0.01, size=theta.shape)  # Add small noise to simulate measurement error
    thetad, cl_robot, finished = cl_planner.move_cl(motion_params=motion_params, t=t, dt=motion_params.dt, thetas=theta)
    if finished:
        break