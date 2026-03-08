import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))
from collections.abc import Generator
import asyncio
from rift.arraytypes import Matrix, Vector
import pyqtgraph
from PySide6 import QtAsyncio
from functools import partial
from rift import steps
from rift import rover
from rift.arraytypes import Matrix
from rift.robot import InverseKinematicsError,TrussRobot
import rift.constrain as cstr
import numpy as np
from rift.transmit.conversion import *


def shrink_in(robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,):
    chassis_mass = np.zeros(len(robot.pos))
    chassis_mass[rover.CHASSIS] = 1.
    chassis_com = cstr.Point.com(chassis_mass)

    dy = 0.1 / resolution
    step_0 = cstr.CompoundConstraint((
        cstr.Motion.lock(rover.CP3),
        cstr.Motion.lock(rover.CQ3),
        cstr.Motion.lock(chassis_com),
        cstr.Motion.make(rover.CL1,y=-dy),
        cstr.Motion.make(rover.CL2,y=-dy),
        cstr.Motion.make(rover.CL3,y=-dy),
        cstr.Motion.make(rover.CR1,y=dy),
        cstr.Motion.make(rover.CR2,y=dy),
        cstr.Motion.make(rover.CR3,y=dy),
    ))
    yield from robot.take_step(step_0, resolution=resolution, allow_redundant=True)

def roll(
    robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[Vector]:
    chassis_midpoints = (
        cstr.Point.avg(rover.CP1, rover.CQ1),
        cstr.Point.avg(rover.CP3, rover.CQ3),
        cstr.Point.avg(rover.CP2, rover.CQ2),
    )
    foot_pairs = (
        (rover.CL1, rover.CR1),
        (rover.CL3, rover.CR3),
        (rover.CL2, rover.CR2),
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
    chassis_mass = np.zeros(len(robot.pos))
    chassis_mass[rover.CHASSIS] = 1.
    chassis_com = cstr.Point.com(chassis_mass)
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
    yield from robot.take_step(step_1, resolution=resolution, allow_redundant=True)
    dx = ((face - feet_midpoint).get(robot.pos)[0] - 0.5*0.875) / resolution
    foot_arc = partial(steps.parabolic, -dx)
    step_2 = cstr.CompoundConstraint((
        cstr.Motion.lock(chassis_com),
        cstr.Motion.make(face - base, z=0.),
        cstr.Motion.make(foot_l, x=dx, z=foot_arc),
        cstr.Motion.make(foot_r, x=dx, z=foot_arc),
    ))
    yield from robot.take_step(step_2, resolution=resolution)
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
    yield from robot.take_step(step_3, resolution=resolution)

def stand(
            robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[Vector]:
    chassis_mass = np.zeros(len(robot.pos))
    chassis_mass[rover.CHASSIS] = 1.
    chassis_com = cstr.Point.com(chassis_mass)

    dz = 0.5 / resolution
    step_0 = cstr.CompoundConstraint((
        cstr.Motion.make(chassis_com,z=dz,x=0,y=0),
        cstr.Motion.make(rover.CP3,x=0,y=0),
        cstr.Motion.make(rover.CL1,z=0),
        cstr.Motion.make(rover.CL2,z=0),
        cstr.Motion.make(rover.CR1,z=0),
        cstr.Motion.make(rover.CR2,z=0),
    ))
    yield from robot.take_step(step_0, resolution=resolution, allow_redundant=True)

def roll_p1(
    robot: TrussRobot,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    base = cstr.Point.avg(rover.CP3, rover.CQ3)
    face = cstr.Point.avg(rover.CP2, rover.CQ2)
    other_feet = [rover.CL2, rover.CR2, rover.CL3, rover.CR3]
    chassis_mass = np.zeros(len(robot.pos))
    chassis_mass[rover.CHASSIS] = 1.
    step_1 = cstr.CompoundConstraint((
        cstr.Motion.lock(base),
        cstr.Orbit.about_y(robot.pos, face-base, np.pi, resolution),
        cstr.Motion.make(face - base, y=0.),
        cstr.Motion.lock(rover.CL1),
        cstr.Motion.lock(rover.CR1),
        *(
            cstr.Motion.make(foot, y=0.)
            for foot in other_feet
        ),
    ))
    yield from robot.take_step(step_1, resolution=resolution, allow_redundant=True)

async def main(
    init_pos: Matrix = rover.ROLLING_POS,
    *,
    resolution: int = 100,
) -> None:
    view, animate = rover.set_up_animation(init_pos)
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    view.show()

    for dq in roll(robot, resolution=resolution):
        stabilizer.update_pos(robot.pos)
        message = f"VEL:{','.join(str(int(c)) for c in (rover.TICKS_PER_SIDE * dq / 1.5).ravel())}"
        print(message)
        animate(stabilizer.pos)
        await asyncio.sleep(0)


    print("Done with animation")


if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main())
