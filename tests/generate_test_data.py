import os

import numpy as np
import pandas as pd
from measure import *
from itertools import product
from multiprocessing import Pool

from rift.truss_config import rover_builder

def tests_from_file(infile, outfile):
    df = pd.DataFrame(pd.read_csv(infile))
    Data_Set_np = df.to_numpy()
    Data_Set_np = Data_Set_np.astype(float)
    X = Data_Set_np[:, :10]
    print(X)
    y = []
    for x in X:
        y.append(list(tests(x[0], x[1], x[2], x[3], x[4], x[5], 20, x[7], x[8], x[9])))
    y = np.array(y)
    data = np.hstack((X, y))
    df = pd.DataFrame(data, columns=['h (ft)',
                                     'P_p (ft)',
                                     'P (ft)',
                                     'theta (deg)',
                                     'w_p (ft)',
                                     'w_f (ft)',
                                     'resolution',
                                     'step_length (ft)',
                                     'roll_rate_limit (ft/s)',
                                     'cycles',
                                     'max_crawl_speed (ft)/s',
                                     'max_foot_lift (ft)/s',
                                     'max_foot_forward (ft)',
                                     'max_step_length (ft)',
                                     'error',
                                     'degen', ])
    df.to_csv(outfile)


def tests(h, P_p, P, theta, w_p, w_f, resolution, step_length, roll_rate_limit, cycles):
    try:
        config = rover_builder(h, P_p, P, theta, w_p, w_f)
    except RuntimeError:
        return None
    except ValueError:
        return h, P_p, P, theta, w_p, w_f, resolution, step_length, roll_rate_limit, cycles,None, None, None, None, None, None
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
        error, degen = None, None
    # vel = run vel test
    # step distance = run distance test
    return h, P_p, P, theta, w_p, w_f, resolution, step_length, roll_rate_limit, cycles, max_crawl_speed, max_foot_lift, max_foot_forward, max_step_length, error, degen

def tests_from_ranges_mp(tofile):
    # h_values = np.arange(.5,3,.1 )
    # P_p_values = np.arange(3, 12.5, .5)
    # P_values = np.array([12])
    # theta_values = np.arange(0, 92.5, 2.5)
    # w_p_values = np.arange(.25, 6.25, .25)
    # w_f_values = np.arange(1, 5, .1)
    # resolution = np.array([100])
    # step_length = np.arange(.5, 1.6, .1)
    # roll_rate_limit = np.array([.13])
    # cycle = np.array([4])

    h_values = np.arange(.5,3,1.75 )
    P_p_values = np.arange(3, 12.5, 5)
    P_values = np.array([12])
    theta_values = np.arange(0, 92.5, 15)
    w_p_values = np.arange(.25, 6.25, 3)
    w_f_values = np.arange(1, 5, 2)
    resolution = np.array([100])
    step_length = np.arange(.5, 1.6, .6)
    roll_rate_limit = np.array([.13])
    cycle = np.array([4])

    combos = product(
        h_values,
        P_p_values,
        P_values,
        theta_values,
        w_p_values,
        w_f_values,
        resolution,
        step_length,
        roll_rate_limit,
        cycle,
    )

    with Pool(processes=os.cpu_count()) as pool:
        results = pool.starmap(tests, combos)
    columns = [
        'h (ft)',
        'P_p (ft)',
        'P (ft)',
        'theta (deg)',
        'w_p (ft)',
        'w_f (ft)',
        'resolution',
        'step_length (ft)',
        'roll_rate_limit (ft/s)',
        'cycles',
        'max_crawl_speed (ft/s)',
        'max_foot_lift (ft/s)',
        'max_foot_forward (ft)',
        'max_step_length (ft)',
        'error',
        'degen'
    ]

    df = pd.DataFrame(results, columns=columns)

    df.to_csv(tofile)


if __name__ == '__main__':
    tests_from_ranges_mp('robot_grid_output.csv')
