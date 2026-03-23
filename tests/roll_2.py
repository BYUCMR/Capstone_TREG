import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))
from collections.abc import Generator
import asyncio
from rift.arraytypes import Matrix, Vector
import pyqtgraph
from PySide6 import QtAsyncio
from functools import partial
from rift import rover
from rift.arraytypes import Matrix
from rift.tubetruss.robots import InverseKinematicsError,TrussRobot
import rift.tubetruss.constrain as cstr
import numpy as np
from rift.transmit.conversion import *
from rift.transmit import commands

def shrink_in(robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,):
    chassis_com = cstr.Point.com(rover.CHASSIS_MASS)

    dy = 0.1 / resolution
    step_0 = cstr.CompoundConstraint((
        cstr.Motion.lock(rover.P3.c),
        cstr.Motion.lock(rover.Q3.c),
        cstr.Motion.lock(chassis_com),
        cstr.Motion.make(rover.L1.c,y=-dy),
        cstr.Motion.make(rover.L2.c,y=-dy),
        cstr.Motion.make(rover.L3.c,y=-dy),
        cstr.Motion.make(rover.R1.c,y=dy),
        cstr.Motion.make(rover.R2.c,y=dy),
        cstr.Motion.make(rover.R3.c,y=dy),
    ))
    yield from robot.repeat_step(step_0, times=resolution, allow_redundant=True)

def roll(
    robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[Vector]:
    chassis_midpoints = (
        cstr.Point.avg(rover.P1.c, rover.Q1.c),
        cstr.Point.avg(rover.P3.c, rover.Q3.c),
        cstr.Point.avg(rover.P2.c, rover.Q2.c),
    )
    foot_pairs = (
        (rover.L1.c, rover.R1.c),
        (rover.L3.c, rover.R3.c),
        (rover.L2.c, rover.R2.c),
    )
    base = chassis_midpoints[i-2]
    face = chassis_midpoints[i-1]
    foot_l, foot_r = foot_pairs[i]
    arm_l, arm_r = foot_pairs[i-2]
    other_feet = [
        foot
        for j, pair in enumerate(foot_pairs)
        for foot in pair
        if j != i
    ]
    chassis_com = cstr.Point.com(rover.CHASSIS_MASS)
    feet_midpoint = cstr.Point.avg(foot_l, foot_r)

    step_1 = cstr.CompoundConstraint((
        cstr.Motion.lock(base),
        cstr.Orbit.about_y(robot.pos, face-base, np.pi, resolution),
        cstr.Motion.make(face - base, y=0.),
        cstr.Motion.lock(foot_l),
        cstr.Motion.lock(foot_r),
        *(
            cstr.Motion.make(foot, y=0.)
            for foot in other_feet
        ),
    ))
    yield from robot.repeat_step(step_1, times=resolution, allow_redundant=True)
    x_dist = (face - feet_midpoint).get(robot.pos)[0] - 0.5*0.875
    step_2 = cstr.CompoundConstraint((
        cstr.Motion.lock(chassis_com),
        cstr.Motion.make(face - base, z=0.),
        cstr.ParabolicPath.make(
            foot_l,
            init_pos=robot.pos,
            delta_x=x_dist,
            resolution=resolution,
        ),
        cstr.ParabolicPath.make(
            foot_r,
            init_pos=robot.pos,
            delta_x=x_dist,
            resolution=resolution,
        ),
    ))
    yield from robot.repeat_step(step_2, times=resolution)
    step_3 = cstr.CompoundConstraint((
        cstr.Motion.lock(face),
        cstr.Orbit.about_y(robot.pos, base-face, np.pi/3, resolution),
        cstr.Motion.make(chassis_com - face, y=0.),
        cstr.Motion.make(base - face, y=0.),
        cstr.Motion.lock(foot_l),
        cstr.Motion.lock(foot_r),
        cstr.Orbit.about_y(robot.pos, arm_l-foot_l, np.pi, resolution),
        cstr.Orbit.about_y(robot.pos, arm_r-foot_r, np.pi, resolution),
    ))
    yield from robot.repeat_step(step_3, times=resolution)

def stand(
            robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
    z=.5
) -> Generator[Vector]:
    chassis_com = cstr.Point.com(rover.CHASSIS_MASS)

    dz = z / resolution
    step_0 = cstr.CompoundConstraint((
        cstr.Motion.make(chassis_com,z=dz,x=0,y=0),
        cstr.Motion.make(rover.P3.c,x=0,y=0),
        cstr.Motion.make(rover.L1.c,z=0),
        cstr.Motion.make(rover.L2.c,z=0),
        cstr.Motion.make(rover.R1.c,z=0),
        cstr.Motion.make(rover.R2.c,z=0),
    ))
    yield from robot.repeat_step(step_0, times=resolution, allow_redundant=True)

def roll_p1(
    robot: TrussRobot,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    base = cstr.Point.avg(rover.P3.c, rover.Q3.c)
    face = cstr.Point.avg(rover.P2.c, rover.Q2.c)
    other_feet = [rover.L2.c, rover.R2.c, rover.L3.c, rover.R3.c]
    step_1 = cstr.CompoundConstraint((
        cstr.Motion.lock(base),
        cstr.Orbit.about_y(robot.pos, face-base, np.pi, resolution),
        cstr.Motion.make(face - base, y=0.),
        cstr.Motion.make(rover.L2.c, z=0),
        cstr.Motion.make(rover.R2.c, z=0),
        cstr.Motion.lock(rover.L1.c),
        cstr.Motion.lock(rover.R1.c),
        *(
            cstr.Motion.make(foot, y=0.)
            for foot in other_feet
        ),
    ))
    yield from robot.repeat_step(step_1, times=resolution, allow_redundant=True)

def roll_p2(
    robot: TrussRobot,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    dz = .2/resolution
    step_1 = cstr.CompoundConstraint((
        cstr.Motion.make(rover.L2.c, z=dz),
        cstr.Motion.make(rover.R2.c, z=dz),
        cstr.Motion.make(rover.L1.c, z=dz),
        cstr.Motion.make(rover.R1.c, z=dz),
        cstr.Motion.lock(rover.P1.c),
        cstr.Motion.lock(rover.Q1.c),
        cstr.Motion.lock(rover.P3.c),
        cstr.Motion.lock(rover.Q3.c),
    ))
    yield from robot.repeat_step(step_1, times=resolution, allow_redundant=True)

def roll_p3(
    robot: TrussRobot,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    dx = -.5/resolution
    step_1 = cstr.CompoundConstraint((

        cstr.Motion.make(rover.L1.c, x=dx,z=0),
        cstr.Motion.make(rover.R1.c,x=dx,z=0),
        cstr.Motion.make(rover.L2.c, x=0),
        cstr.Motion.make(rover.R2.c,x=0),
        cstr.Motion.lock(rover.P1.c),
        cstr.Motion.lock(rover.Q1.c),
        cstr.Motion.lock(rover.P3.c),
        cstr.Motion.lock(rover.Q3.c),
    ))
    yield from robot.repeat_step(step_1, times=resolution, allow_redundant=True)

def roll_p4(
    robot: TrussRobot,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    dz = -.2/resolution
    step_1 = cstr.CompoundConstraint((
        cstr.Motion.make(rover.L1.c, z=dz),
        cstr.Motion.make(rover.R1.c, z=dz),
        cstr.Motion.lock(rover.P1.c),
        cstr.Motion.lock(rover.Q1.c),
        cstr.Motion.lock(rover.P3.c),
        cstr.Motion.lock(rover.Q3.c),
    ))
    yield from robot.repeat_step(step_1, times=resolution, allow_redundant=True)

def scoot(
    robot: TrussRobot,
    *,
    resolution: int = 100,
)-> Generator[Vector]:
    chassis_com = cstr.Point.com(rover.CHASSIS_MASS)

    dx = -0.2 / resolution
    step = cstr.CompoundConstraint((
        cstr.Motion.make(chassis_com,z=0,x=dx,y=0),
        cstr.Motion.make(rover.L1.c,z=0),
        cstr.Motion.make(rover.L2.c,z=0,x=0),
        cstr.Motion.make(rover.R1.c,z=0),
        cstr.Motion.lock(rover.R2.c),
    ))
    yield from robot.repeat_step(step, times=resolution, allow_redundant=True)

def roll_p5(
    robot: TrussRobot,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    chassis_com = cstr.Point.com(rover.CHASSIS_MASS)
    base = cstr.Point.avg(rover.P3.c, rover.Q3.c)
    face = cstr.Point.avg(rover.P2.c, rover.Q2.c)
    step = cstr.CompoundConstraint((
        cstr.Motion.lock(face),
        cstr.Orbit.about_y(robot.pos, base-face, np.pi/3, resolution),
        cstr.Motion.make(chassis_com - face, y=0.),
        cstr.Motion.make(base - face, y=0.),
        cstr.Motion.lock(rover.R2.c),
        cstr.Motion.lock(rover.L2.c),
        cstr.Orbit.about_y(robot.pos, rover.L1.c-rover.L2.c, np.pi, resolution),
        cstr.Orbit.about_y(robot.pos, rover.L1.c-rover.L2.c, np.pi, resolution),
    ))
    yield from robot.repeat_step(step, times=resolution, allow_redundant=True)

async def main(
    init_pos: Matrix = rover.ROLLING_POS,
    *,
    resolution: int = 100,
) -> None:
    view, animate = rover.set_up_animation(init_pos)
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    view.show()

    for dq in scoot(robot, resolution=100):
        stabilizer.update_pos(robot.pos)
        cmd =rover.TICKS_PER_SIDE * dq / 1.5
        print(cmd.astype(int))
        message = commands.bVEL(cmd.astype(int),1.5)
        # message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
        print(message)
        animate(stabilizer.pos)
        await asyncio.sleep(0)

    # for dq in roll_p1(robot, resolution=100):
    #     stabilizer.update_pos(robot.pos)
    #     message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
    #     print(message)
    #     animate(stabilizer.pos)
    #     await asyncio.sleep(0)

    # for dq in roll_p2(robot, resolution=100):
    #     stabilizer.update_pos(robot.pos)
    #     message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
    #     print(message)
    #     animate(stabilizer.pos)
    #     await asyncio.sleep(0)

    # for dq in roll_p3(robot, resolution=100):
    #     stabilizer.update_pos(robot.pos)
    #     message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
    #     print(message)
    #     animate(stabilizer.pos)
    #     await asyncio.sleep(0)

    # for dq in roll_p4(robot, resolution=100):
    #     stabilizer.update_pos(robot.pos)
    #     message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
    #     print(message)
    #     animate(stabilizer.pos)
    #     await asyncio.sleep(0)

    # for dq in roll_p5(robot, resolution=100):
    #     stabilizer.update_pos(robot.pos)
    #     message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
    #     print(message)
    #     animate(stabilizer.pos)
        # await asyncio.sleep(0)




    print("Done with animation")



if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main())
