import numpy as np
import pandas as pd
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

h_grid, P_p_grid, P_grid, theta_grid, w_p_grid, w_f_grid,res_grid,step_length_grid = np.meshgrid(h_values, P_p_values, P_values, theta_values,
                                                                       w_p_values, w_f_values,resolution,step_length, indexing='ij')

h_flat = h_grid.ravel()
P_p_flat = P_p_grid.ravel()
P_flat = P_grid.ravel()
theta_flat = theta_grid.ravel()
w_p_flat = w_p_grid.ravel()
w_f_flat = w_f_grid.ravel()
res_flat = res_grid.ravel()
step_flat = step_length_grid.ravel()


def tests(h,P_p,P,theta,w_p,w_f,resolution,step_length):
    try:
        config = rover_builder(h, P_p, P, theta, w_p, w_f)
    except RuntimeError:
        return None
    except ValueError:
        return None
    #vel = run vel test
    #step distance = run distance test

test_vec = np.vectorize(tests)
speed = test_vec(h_flat,P_p_flat,P_flat,theta_flat,w_p_flat,w_f_flat,resolution,step_length)

df = pd.DataFrame({'h':h_flat,
                   'P_p':P_p_flat,
                   'P_flat':P_flat,
                   'theta_flat':theta_flat,
                   'w_p_flat':w_p_flat,
                   'w_f_flat':w_f_flat,
                   'resolution':resolution,
                   'step_length':step_length,
                   'speed':speed})

df.to_csv('robot_tests.csv')