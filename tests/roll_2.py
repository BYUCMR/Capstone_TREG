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
from rift.robot import InverseKinematicsError,RobotInverse
import rift.constrain as cstr
import numpy as np
from rift.transmit.conversion import *


def shrink_in(robot: RobotInverse,
    *,
    i: int = 0,
    resolution: int = 100,):
    payload_mass = np.zeros(len(robot.pos))
    payload_mass[rover.PAYLOAD] = 1.
    payload_com = cstr.Point.com(payload_mass)

    dy = 0.1 / resolution
    step_0 = cstr.CompoundConstraint((
        cstr.Motion.lock(rover.CPL3),
        cstr.Motion.lock(rover.CPR3),
        cstr.Motion.lock(payload_com),
        cstr.Motion.make(rover.CL1,y=-dy),
        cstr.Motion.make(rover.CL2,y=-dy),
        cstr.Motion.make(rover.CL3,y=-dy),
        cstr.Motion.make(rover.CR1,y=dy),
        cstr.Motion.make(rover.CR2,y=dy),
        cstr.Motion.make(rover.CR3,y=dy),
    ))
    yield from robot.take_step(step_0, resolution=resolution, allow_redundant=True)
    
def roll(
    robot: RobotInverse,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[tuple[Matrix, Vector]]:
    payload_midpoints = (
        cstr.Point.avg(rover.CPL1, rover.CPR1),
        cstr.Point.avg(rover.CPL3, rover.CPR3),
        cstr.Point.avg(rover.CPL2, rover.CPR2),
    )
    foot_pairs = (
        (rover.CL1, rover.CR1),
        (rover.CL3, rover.CR3),
        (rover.CL2, rover.CR2),
    )
    base = payload_midpoints[i-2]
    face = payload_midpoints[i-1]
    foot_l, foot_r = foot_pairs[i]
    arm_l, arm_r = foot_pairs[i-2]
    other_feet = [
        foot
        for j, pair in enumerate(foot_pairs)
        for foot in pair
        if j != i
    ]
    payload_mass = np.zeros(len(robot.pos))
    payload_mass[rover.PAYLOAD] = 1.
    payload_com = cstr.Point.com(payload_mass)
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
        cstr.Motion.lock(payload_com),
        cstr.Motion.make(face - base, z=0.),
        cstr.Motion.make(foot_l, x=dx, z=foot_arc),
        cstr.Motion.make(foot_r, x=dx, z=foot_arc),
    ))
    yield from robot.take_step(step_2, resolution=resolution)
    step_3 = cstr.CompoundConstraint((
        cstr.Motion.lock(face),
        cstr.Orbit.about_y(robot.pos, base-face, np.pi/3, resolution),
        cstr.Motion.make(payload_com - face, y=0.),
        cstr.Motion.make(base - face, y=0.),
        cstr.Motion.lock(foot_l),
        cstr.Motion.lock(foot_r),
        cstr.Orbit.about_y(robot.pos, arm_l-foot_l, np.pi, resolution),
        cstr.Orbit.about_y(robot.pos, arm_r-foot_r, np.pi, resolution),
    ))
    yield from robot.take_step(step_3, resolution=resolution)

def stand(
            robot: RobotInverse,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[tuple[Matrix, Vector]]:
    payload_mass = np.zeros(len(robot.pos))
    payload_mass[rover.PAYLOAD] = 1.
    payload_com = cstr.Point.com(payload_mass)

    dz = 0.5 / resolution
    step_0 = cstr.CompoundConstraint((
        cstr.Motion.make(payload_com,z=dz,x=0,y=0),
        cstr.Motion.make(rover.CPL3,x=0,y=0),
        cstr.Motion.make(rover.CL1,z=0),
        cstr.Motion.make(rover.CL2,z=0),
        cstr.Motion.make(rover.CR1,z=0),
        cstr.Motion.make(rover.CR2,z=0),
    ))
    yield from robot.take_step(step_0, resolution=resolution, allow_redundant=True)

def roll_p1(
    robot: RobotInverse,
    *,
    resolution: int = 100,
) -> Generator[tuple[Matrix, Vector]]:
    base = cstr.Point.avg(rover.CPL3, rover.CPR3)
    face = cstr.Point.avg(rover.CPL2, rover.CPR2)
    other_feet = [rover.CL2, rover.CR2, rover.CL3, rover.CR3]
    payload_mass = np.zeros(len(robot.pos))
    payload_mass[rover.PAYLOAD] = 1.
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
    animator = rover.make_animator(init_pos)
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    positions = asyncio.Queue[Matrix](resolution)

    async def crawl() -> None:
        for _,dq in roll(robot, resolution=resolution):
            stabilizer.update_pos(robot.pos)
            # await positions.put(robot.pos.copy())
            
            message = f"VEL:{','.join(str(int(c)) for c in ticks_to_tps(dist_to_ticks(6,dq)).ravel())}"
            print(message)
            await positions.put(stabilizer.pos)
        # for _ in range(3):
        #     for _ in rover.crawl(robot, resolution=resolution):
        #         # stabilizer.update_pos(robot.pos)
        #         await positions.put(robot.pos.copy())


    crawling_task = asyncio.create_task(crawl())
    animation_task = asyncio.create_task(animator.animate(positions))
    try:
        await crawling_task
    except InverseKinematicsError as e:
        print(e.args[0])
    print("Done with IK")
    positions.shutdown()
    await animation_task
    print("Done with animation")


if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main())
