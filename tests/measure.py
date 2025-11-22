import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

from functools import partial
from itertools import pairwise

import numpy as np

from rift.gentools import expend
from rift.robot import RobotInverse, SingularityError
from rift.steps import make_step_array, parabola
from rift.truss_config import TrussConfig


def measure_max_crawl_speed(
    config: TrussConfig,
    *,
    step_length: float = 0.8,
    roll_rate_limit: float,
    cycles: int = 1,
    resolution: int,
) -> np.floating:
    robot = RobotInverse(config)
    start_x = np.max(robot.pos[:, 0])
    rolls = [robot.roll]
    for i in range(cycles):
        for path in robot.crawl(step_length, resolution=resolution):
            rolls.append(robot.roll)
    d_rolls = np.array([b - a for a, b in pairwise(rolls)])
    end_x = np.max(robot.pos[:, 0])
    delta_x = end_x - start_x
    min_dt = np.max(d_rolls, axis=1) / roll_rate_limit
    max_speed = delta_x / np.sum(min_dt)
    return max_speed


def measure_max_foot_lift(config: TrussConfig, *, dz: float = 0.01) -> float:
    robot = RobotInverse(config)
    z0 = robot.pos[0, 2]
    motion = np.full_like(robot.pos, np.nan)
    motion[0] = [0., 0., dz]
    motion[1] = motion[6] = motion[7] = 0.
    while True:
        try:
            robot.take_substep(motion)
        except SingularityError:
            break
    return robot.pos[0, 2] - dz - z0


def measure_max_foot_forward(config: TrussConfig, *, dx: float = 0.01) -> float:
    robot = RobotInverse(config)
    x0 = robot.pos[0, 0]
    motion = np.full_like(robot.pos, np.nan)
    motion[0] = [dx, 0., 0.]
    motion[1] = motion[6] = motion[7] = 0.
    while True:
        try:
            robot.take_substep(motion)
        except SingularityError:
            break
    return robot.pos[0, 0] - dx - x0


def measure_max_step_length(config: TrussConfig, *, dx: float = 0.01, resolution: int) -> float:
    robot = RobotInverse(config)
    initial_state = robot.state
    initial_rigidity = robot.rigidity
    step_length = dx
    while True:
        arc = partial(parabola, d=step_length)
        step = make_step_array(
            robot.pos.shape,
            (0, arc),
            (1, 0.),
            (6, 0.),
            (7, 0.),
            resolution=resolution,
        )
        try:
            expend(robot.take_step(step))
        except SingularityError:
            break
        step_length += dx
        robot.state = initial_state
        robot.rigidity = initial_rigidity
    return step_length - dx


def measure_length_change(config: TrussConfig, *, cycles: int = 1, resolution: int) -> tuple[float, float]:
    robot = RobotInverse(config)
    d0 = np.array([robot.pos[i] - robot.pos[j] for i, j in robot.structure.links])
    L0 = np.sqrt(np.sum(np.square(d0), axis=1))
    for _ in range(cycles):
        expend(robot.crawl(resolution=resolution))
    d1 = np.array([robot.pos[i] - robot.pos[j] for i, j in robot.structure.links])
    L1 = np.sqrt(np.sum(np.square(d1), axis=1))
    delta_L = L1 - L0
    error = np.abs(np.sum(delta_L))
    degen = np.sum(np.abs(delta_L))
    return error, degen


def main() -> None:
    from rift.truss_config import CONFIG_ROVER
    cycles = 1
    resolution = 100
    roll_rate_limit = 0.13
    step_length = 0.8
    max_crawl_speed = measure_max_crawl_speed(
        CONFIG_ROVER,
        step_length=step_length,
        roll_rate_limit=roll_rate_limit,
        cycles=cycles,
        resolution=resolution,
    )
    dz = 0.005
    dx = 0.005
    ds = 0.1
    max_foot_lift = measure_max_foot_lift(CONFIG_ROVER, dz=dz)
    max_foot_forward = measure_max_foot_forward(CONFIG_ROVER, dx=dx)
    max_step_length = measure_max_step_length(CONFIG_ROVER, dx=ds, resolution=resolution)
    error, degen = measure_length_change(CONFIG_ROVER, cycles=cycles, resolution=resolution)
    print(f"Walk cycles:...............{cycles} sets of 4 steps")
    print(f"Resolution:................{resolution} substeps per step")
    print(f"Step Length:...............{step_length:.3g} ft")
    print(f"Roll rate limit:...........{roll_rate_limit:.3g} ft/s")
    print(f"Maximum crawl speed:.......{max_crawl_speed:.3g} ft/s")
    print(f"Maximum foot lift:.........{max_foot_lift:.3g}±{dz:.3g} ft")
    print(f"Maximum foot forward:......{max_foot_forward:.3g}±{dx:.3g} ft")
    print(f"Maximum step length:.......{max_step_length:.3g}±{ds:.3g} ft")
    print(f"Shape error |ΣΔL|:.........{error:.3g} ft")
    print(f"Shape degeneration Σ|ΔL|:..{degen:.3g} ft")


if __name__ == '__main__':
    main()
