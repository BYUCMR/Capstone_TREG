import numpy as np
import pandas as pd
from measure import *
from cvxpy import length

from rift.truss_config import rover_builder

h_values = np.array(list(range(.5, )))
P_p_values = np.array(list(range(3, 12.5, .5)))
P_values = np.array([12])
theta_values = np.array(list(range(0, 92.5, 2.5)))
w_p_values = np.array(list(range(.25, 6.25, .25)))
w_f_values = np.array(list(range(1, 5, .1)))
resolution = np.array([100])
step_length = np.array(list(range(.5, 1.6, .1)))
roll_rate_limit = np.array([.13])
cycle = np.array([4])

h_grid, P_p_grid, P_grid, theta_grid, w_p_grid, w_f_grid,res_grid,step_length_grid,roll_rate_limit_grid,cycle_grid = np.meshgrid(h_values, P_p_values, P_values, theta_values,
                                                                       w_p_values, w_f_values,resolution,step_length,roll_rate_limit,cycle, indexing='ij')

h_flat = h_grid.ravel()
P_p_flat = P_p_grid.ravel()
P_flat = P_grid.ravel()
theta_flat = theta_grid.ravel()
w_p_flat = w_p_grid.ravel()
w_f_flat = w_f_grid.ravel()
res_flat = res_grid.ravel()
step_flat = step_length_grid.ravel()
roll_rate_limit_flat = roll_rate_limit_grid.ravel()
cycle_falt = cycle_grid.ravel()


def tests(h,P_p,P,theta,w_p,w_f,resolution,step_length,roll_rate_limit,cycles):
    try:
        config = rover_builder(h, P_p, P, theta, w_p, w_f)
    except RuntimeError:
        return None
    except ValueError:
        return None,None,None,None,None,None
    dz = 0.005
    dx = 0.005
    ds = 0.1
    try:
        max_crawl_speed = measure_max_crawl_speed(
            config,
            step_length=step_length,
            roll_rate_limit=roll_rate_limit,
            cycles=cycles,
            resolution=resolution,
        )
    except SingularityError:
        max_crawl_speed = None
    try:
        max_foot_lift = measure_max_foot_lift(config, dz=dz)
    except SingularityError:
        max_foot_lift = None
    try:
        max_foot_forward = measure_max_foot_forward(config, dx=dx)
    except SingularityError:
        max_foot_forward = None
    try:
        max_step_length = measure_max_step_length(config, dx=ds, resolution=resolution)
    except SingularityError:
        max_step_length = None
    try:
        error, degen = measure_length_change(config, cycles=cycles, resolution=resolution)
    except SingularityError:
        error,degen = None,None
    #vel = run vel test
    #step distance = run distance test
    return max_crawl_speed,max_foot_lift,max_foot_forward,max_step_length,error, degen

test_vec = np.vectorize(tests)
max_crawl_speed, max_foot_lift, max_foot_forward, max_step_length, error, degen = test_vec(h_flat,P_p_flat,P_flat,theta_flat,w_p_flat,w_f_flat,resolution,step_length)

df = pd.DataFrame({'h (ft)':h_flat,
                   'P_p (ft)':P_p_flat,
                   'P (ft)':P_flat,
                   'theta (deg)':theta_flat,
                   'w_p (ft)':w_p_flat,
                   'w_f (ft)':w_f_flat,
                   'resolution':resolution,
                   'step_length (ft)':step_length,
                   'roll_rate_limit (ft/s)':roll_rate_limit,
                   'cycles':cycle,
                   'max_crawl_speed (ft)':max_crawl_speed,
                   'max_foot_lift (ft)/s':max_foot_lift,
                   'max_foot_forward (ft)':max_foot_forward,
                   'max_step_length (ft)':max_step_length,
                   'error':error,
                   'degen':degen,})

df.to_csv('robot_tests.csv')