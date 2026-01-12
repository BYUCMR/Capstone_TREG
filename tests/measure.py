import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import math

import numpy as np

import rift.indices as I
from rift.arraytypes import Matrix, MatrixStack
from rift.grav import Stabilizer
from rift.robot import InverseKinematicsError, RobotInverse
from rift.steps import make_step_array, parabola
from rift.truss_config import TrussConfig


def record_motion(
    config: TrussConfig,
    *,
    step_length: float = 0.8,
    cycles: int = 1,
    resolution: int,
) -> tuple[MatrixStack, MatrixStack, Matrix]:
    robot = RobotInverse.from_config(config)
    n = 4 * cycles * resolution
    pos = np.zeros((n + 1, *robot.pos.shape))
    d_pos = np.zeros((n, *robot.pos.shape))
    d_roll = np.zeros((n, len(robot.structure.incidence_inv)))
    pos[0] = robot.pos
    for i, (dx, dr) in enumerate(robot.crawl(
        cycles, step_length, resolution=resolution
    )):
        pos[i + 1] = robot.pos
        d_pos[i] = dx
        d_roll[i] = dr
    return pos, d_pos, d_roll


def measure_max_incline(config: TrussConfig, *, da: float = 1.) -> float:
    da = math.radians(da)
    robot = RobotInverse.from_config(config)
    stabilizer = Stabilizer.from_config(config)
    angle = 0.
    while True:
        stabilizer.gravity = np.array([-math.sin(angle), 0., -math.cos(angle)])
        if stabilizer.adjust_for(robot.pos) or angle >= 0.5*np.pi:
            break
        angle += da
    return math.degrees(angle)


def measure_stable_substeps(config: TrussConfig, pos_hist: MatrixStack) -> int:
    stabilizer = Stabilizer.from_config(config)
    for i in range(len(pos_hist) - 1):
        stabilizer.source_pos = pos_hist[i]
        if stabilizer.adjust_for(pos_hist[i+1]):
            return i
    return len(pos_hist)


def measure_max_crawl_speed(
    d_rolls: Matrix,
    *,
    step_length: float = 0.8,
    roll_rate_limit: float,
    cycles: int = 1,
) -> float:
    min_dt = np.max(d_rolls, axis=1) / roll_rate_limit
    max_speed = cycles * step_length / np.sum(min_dt)
    return float(max_speed)


def measure_max_foot_lift(config: TrussConfig, *, dz: float = 0.01) -> float:
    robot = RobotInverse.from_config(config)
    z0 = robot.pos[I.L1, 2]
    motion = np.full_like(robot.pos, np.nan)
    motion[I.L1] = [0., 0., dz]
    motion[I.L2] = motion[I.R1] = motion[I.R2] = 0.
    while True:
        try:
            robot.take_substep(motion)
        except InverseKinematicsError:
            break
    return robot.pos[I.L1, 2] - dz - z0


def measure_max_foot_forward(config: TrussConfig, *, dx: float = 0.01) -> float:
    robot = RobotInverse.from_config(config)
    x0 = robot.pos[I.L1, 0]
    motion = np.full_like(robot.pos, np.nan)
    motion[I.L1] = [dx, 0., 0.]
    motion[I.L2] = motion[I.R1] = motion[I.R2] = 0.
    while True:
        try:
            robot.take_substep(motion)
        except InverseKinematicsError:
            break
    return robot.pos[I.L1, 0] - dx - x0


def measure_max_step_length(config: TrussConfig, *, dx: float = 0.01, resolution: int) -> float:
    robot = RobotInverse.from_config(config)
    initial_pos = robot.pos.copy()
    step_length = dx
    step = make_step_array(
        robot.pos.shape,
        (I.L2, 0.),
        (I.R1, 0.),
        (I.R2, 0.),
        resolution=resolution,
    )
    t = np.linspace(0., 1., resolution)
    while True:
        step[:, I.L1, :] = parabola(t, d=step_length)
        try:
            for _ in robot.take_step(step):
                pass
        except InverseKinematicsError:
            break
        step_length += dx
        robot.pos = initial_pos.copy()
    return step_length - dx


def measure_length_change(config: TrussConfig, pos_hist: MatrixStack) -> tuple[float, float]:
    robot = RobotInverse.from_config(config)
    p0 = pos_hist[0]
    d0 = np.array([p0[i] - p0[j] for i, j in robot.structure.links])
    L0 = np.sqrt(np.sum(np.square(d0), axis=1))
    p1 = pos_hist[-1]
    d1 = np.array([p1[i] - p1[j] for i, j in robot.structure.links])
    L1 = np.sqrt(np.sum(np.square(d1), axis=1))
    delta_L = L1 - L0
    error = np.abs(np.sum(delta_L))
    degen = np.sum(np.abs(delta_L))
    return error, degen


def main() -> None:
    from rift.truss_config import ROVER_CONFIG
    cycles = 1
    resolution = 100
    roll_rate_limit = 0.13
    step_length = 0.8
    da = 1.
    max_incline = measure_max_incline(ROVER_CONFIG, da=da)
    pos_hist, d_pos_hist, d_roll_hist = record_motion(
        ROVER_CONFIG,
        step_length=step_length,
        cycles=cycles,
        resolution=resolution,
    )
    stable_substeps = measure_stable_substeps(ROVER_CONFIG, pos_hist)
    max_crawl_speed = measure_max_crawl_speed(
        d_roll_hist,
        step_length=step_length,
        roll_rate_limit=roll_rate_limit,
        cycles=cycles,
    )
    dz = 0.005
    dx = 0.005
    ds = 0.1
    max_foot_lift = measure_max_foot_lift(ROVER_CONFIG, dz=dz)
    max_foot_forward = measure_max_foot_forward(ROVER_CONFIG, dx=dx)
    max_step_length = measure_max_step_length(ROVER_CONFIG, dx=ds, resolution=resolution)
    error, degen = measure_length_change(ROVER_CONFIG, pos_hist)
    print(f"Walk cycles:...............{cycles} sets of 4 steps")
    print(f"Resolution:................{resolution} substeps per step")
    print(f"Step Length:...............{step_length:.3g} ft")
    print(f"Maximum incline:...........{max_incline:.0f}°")
    print(f"Stable substeps:...........{stable_substeps} substeps")
    print(f"Roll rate limit:...........{roll_rate_limit:.3g} ft/s")
    print(f"Maximum crawl speed:.......{max_crawl_speed:.3g} ft/s")
    print(f"Maximum foot lift:.........{max_foot_lift:.3g}±{dz:.3g} ft")
    print(f"Maximum foot forward:......{max_foot_forward:.3g}±{dx:.3g} ft")
    print(f"Maximum step length:.......{max_step_length:.3g}±{ds:.3g} ft")
    print(f"Shape error |ΣΔL|:.........{error:.3g} ft")
    print(f"Shape degeneration Σ|ΔL|:..{degen:.3g} ft")


if __name__ == '__main__':
    main()
