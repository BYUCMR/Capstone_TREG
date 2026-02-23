import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import csv
import multiprocessing as mp
import time
from collections.abc import Callable
from itertools import product

import numpy as np

import rift.rover

import measure


def or_nan[**P](f: Callable[P, float], *args: P.args, **kwargs: P.kwargs) -> float:
    try:
        result = f(*args, **kwargs)
    except Exception:
        return np.nan
    else:
        return result


def tests_from_file(infile, outfile):
    import pandas as pd
    df = pd.DataFrame(pd.read_csv(infile))
    Data_Set_np = df.to_numpy()
    Data_Set_np = Data_Set_np.astype(float)
    X = Data_Set_np[:, :10]
    print(X)
    y = []
    for x in X:
        y.append(list(tests(x[0], x[1], x[2], x[3], x[4], x[5], 20, x[7], x[8])))
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


def tests(h, d_p, theta, w_p, d_f, resolution, step_length, roll_rate_limit, cycles):
    try:
        config = rift.rover.make_pos(h, d_p, theta, w_p, d_f)
    except Exception:
        return [np.nan] * 9
    max_incline = or_nan(measure.measure_max_incline, config)
    try:
        pos_hist, d_pos_hist, d_roll_hist = measure.record_motion(
            config,
            step_length=step_length,
            cycles=cycles,
            resolution=resolution,
        )
    except Exception:
        stable_substeps = np.nan
        max_shoulder_angle = np.nan
        max_crawl_speed = np.nan
        error = np.nan
        degen = np.nan
    else:
        stable_substeps = or_nan(measure.measure_stable_substeps, config, pos_hist)
        max_shoulder_angle = or_nan(measure.measure_max_shoulder_angle, pos_hist)
        max_crawl_speed = or_nan(
            measure.measure_max_crawl_speed,
            d_roll_hist,
            step_length=step_length,
            roll_rate_limit=roll_rate_limit,
            cycles=cycles,
        )
        try:
            error, degen = measure.measure_length_change(config, pos_hist)
        except Exception:
            error, degen = np.nan, np.nan
    max_foot_lift = or_nan(measure.measure_max_foot_lift, config)
    max_foot_forward = or_nan(measure.measure_max_foot_forward, config)
    max_step_length = or_nan(measure.measure_max_step_length, config, dx=0.1, resolution=resolution)
    return [
        max_incline,
        stable_substeps,
        max_shoulder_angle,
        max_crawl_speed,
        max_foot_lift,
        max_foot_forward,
        max_step_length,
        error,
        degen,
    ]


def tests_from_ranges_mp():
    roll_rate_limit = 0.0325
    cycles = 1
    resolution = 100

    height = np.arange(0.125, 0.75, 0.125)
    payload_length = np.arange(0.25, 1.0, 0.15)
    shoulder_angle = np.arange(0, 100, 10)
    payload_width = np.arange(0.0625, 1.5625, 0.25)
    foot_width = np.arange(0.25, 1.25, 0.125)
    step_length = np.arange(0.125, 0.375, 0.125)

    inputs = list(product(
        height,
        payload_length,
        shoulder_angle,
        payload_width,
        foot_width,
        [resolution],
        step_length,
        [roll_rate_limit],
        [cycles],
    ))

    with mp.Pool() as pool:
        outputs = pool.starmap(tests, inputs)

    columns = [
        'h (ft)',
        'P_p (ft)',
        'theta (deg)',
        'w_p (ft)',
        'w_f (ft)',
        'resolution',
        'step_length (ft)',
        'roll_rate_limit (ft/s)',
        'cycles',
        'maximum_incline (deg)',
        'stable_substeps',
        'max_shoulder_angle (deg)',
        'max_crawl_speed (ft/s)',
        'max_foot_lift (ft/s)',
        'max_foot_forward (ft)',
        'max_step_length (ft)',
        'error',
        'degen'
    ]
    data = [[*i, *o] for i, o in zip(inputs, outputs, strict=True)]
    return columns, data


def main(tofile):
    t0 = time.time()
    columns, data = tests_from_ranges_mp()
    duration = (time.time() - t0) / 60.
    print(f"Tests took {duration:.2f} minutes to run.")
    with open(tofile, 'w') as file:
        writer = csv.writer(file, lineterminator='\n')
        writer.writerow(columns)
        writer.writerows(data)


if __name__ == '__main__':
    main('robot_grid_output.csv')
