import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import math
from functools import partial

import numpy as np

from rift import constrain as cstr
from rift import rover
from rift import steps
from rift.arraytypes import Matrix, MatrixStack
from rift.robot import InverseKinematicsError


def record_motion(
    init_pos: Matrix,
    *,
    step_length: float = 0.125,
    cycles: int = 1,
    resolution: int,
) -> tuple[MatrixStack, MatrixStack, Matrix]:
    robot = rover.make_robot(init_pos)
    n = 4 * cycles * resolution
    pos = np.zeros((n + 1, *robot.pos.shape))
    d_pos = np.zeros((n, *robot.pos.shape))
    d_roll = np.zeros((n, len(robot.length_to_roll)))
    pos[0] = robot.pos
    for i, (dx, dr) in enumerate(rover.crawl(
        robot, cycles, step_length, resolution=resolution
    )):
        pos[i + 1] = robot.pos
        d_pos[i] = dx
        d_roll[i] = dr
    return pos, d_pos, d_roll


def measure_max_incline(init_pos: Matrix, *, da: float = 1.) -> float:
    da = math.radians(da)
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    angle = 0.
    while True:
        stabilizer.gravity = np.array([-math.sin(angle), 0., -math.cos(angle)])
        if stabilizer.adjust_for(robot.pos) or angle >= 0.5*np.pi:
            break
        angle += da
    return math.degrees(angle)


def measure_stable_substeps(init_pos: Matrix, pos_hist: MatrixStack) -> int:
    stabilizer = rover.make_stabilizer(init_pos)
    for i in range(len(pos_hist) - 1):
        stabilizer.source_pos = pos_hist[i]
        if stabilizer.adjust_for(pos_hist[i+1]):
            return i
    return len(pos_hist)


def measure_max_crawl_speed(
    d_rolls: Matrix,
    *,
    step_length: float = 0.125,
    roll_rate_limit: float,
    cycles: int = 1,
) -> float:
    min_dt = np.max(d_rolls, axis=1) / roll_rate_limit
    max_speed = cycles * step_length / np.sum(min_dt)
    return float(max_speed)


def measure_max_foot_lift(init_pos: Matrix, *, dz: float = 0.0025) -> float:
    robot = rover.make_robot(init_pos)
    z0 = robot.pos[rover.L1, 2]
    constraint = cstr.CompoundConstraint((
        cstr.Motion.make(rover.CL1, 0., 0., dz),
        cstr.Motion.lock(rover.CL2),
        cstr.Motion.lock(rover.CR1),
        cstr.Motion.lock(rover.CR2),
    ))
    while True:
        try:
            robot.take_substep(constraint)
        except InverseKinematicsError:
            break
    return robot.pos[rover.L1, 2] - dz - z0


def measure_max_foot_forward(init_pos: Matrix, *, dx: float = 0.0025) -> float:
    robot = rover.make_robot(init_pos)
    x0 = robot.pos[rover.L1, 0]
    constraint = cstr.CompoundConstraint((
        cstr.Motion.make(rover.CL1, dx, 0., 0.),
        cstr.Motion.lock(rover.CL2),
        cstr.Motion.lock(rover.CR1),
        cstr.Motion.lock(rover.CR2),
    ))
    while True:
        try:
            robot.take_substep(constraint)
        except InverseKinematicsError:
            break
    return robot.pos[rover.L1, 0] - dx - x0


def measure_max_step_length(init_pos: Matrix, *, dx: float = 0.0025, resolution: int) -> float:
    robot = rover.make_robot(init_pos)
    initial_pos = robot.pos.copy()
    step_length = dx
    t = np.linspace(0., 1., resolution)
    while True:
        k = step_length / len(t)
        constraint = cstr.CompoundConstraint((
            cstr.Motion.make(rover.CL1, k, 0., partial(steps.parabolic, k)),
            cstr.Motion.lock(rover.CL2),
            cstr.Motion.lock(rover.CR1),
            cstr.Motion.lock(rover.CR2),
        ))
        try:
            for ti in t:
                robot.take_substep(constraint, t=ti)
        except InverseKinematicsError:
            break
        step_length += dx
        robot.pos = initial_pos.copy()
    return step_length - dx


def measure_length_change(init_pos: Matrix, pos_hist: MatrixStack) -> tuple[float, float]:
    robot = rover.make_robot(init_pos)
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
    init_pos = rover.CRAWLING_POS
    cycles = 1
    resolution = 100
    roll_rate_limit = 0.0325
    step_length = 0.125
    da = 1.
    max_incline = measure_max_incline(init_pos, da=da)
    pos_hist, d_pos_hist, d_roll_hist = record_motion(
        init_pos,
        step_length=step_length,
        cycles=cycles,
        resolution=resolution,
    )
    stable_substeps = measure_stable_substeps(init_pos, pos_hist)
    max_crawl_speed = measure_max_crawl_speed(
        d_roll_hist,
        step_length=step_length,
        roll_rate_limit=roll_rate_limit,
        cycles=cycles,
    )
    dz = 0.00125
    dx = 0.00125
    ds = 0.025
    max_foot_lift = measure_max_foot_lift(init_pos, dz=dz)
    max_foot_forward = measure_max_foot_forward(init_pos, dx=dx)
    max_step_length = measure_max_step_length(init_pos, dx=ds, resolution=resolution)
    error, degen = measure_length_change(init_pos, pos_hist)
    print(f"Tube Length:...............{4*3} ft")
    print(f"Walk cycles:...............{cycles} sets of 4 steps")
    print(f"Resolution:................{resolution} substeps per step")
    print(f"Step Length:...............{4*step_length:.3g} ft")
    print(f"Maximum incline:...........{max_incline:.0f}°")
    print(f"Stable substeps:...........{stable_substeps} substeps")
    print(f"Roll rate limit:...........{4*roll_rate_limit:.3g} ft/s")
    print(f"Maximum crawl speed:.......{4*max_crawl_speed:.3g} ft/s")
    print(f"Maximum foot lift:.........{4*max_foot_lift:.3g}±{4*dz:.3g} ft")
    print(f"Maximum foot forward:......{4*max_foot_forward:.3g}±{4*dx:.3g} ft")
    print(f"Maximum step length:.......{4*max_step_length:.3g}±{4*ds:.3g} ft")
    print(f"Shape error |ΣΔL|:.........{4*error:.3g} ft")
    print(f"Shape degeneration Σ|ΔL|:..{4*degen:.3g} ft")


if __name__ == '__main__':
    main()
