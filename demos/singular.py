import asyncio
from collections.abc import Generator

import pyqtgraph
from PySide6 import QtAsyncio

from rift import anim, rover
from rift.arraytypes import Matrix
from rift.motion import constraints as cstr, steps
from rift.tubetruss.robotics import TrussController
from rift.tubetruss.singularity import SingularityCost


def sit() -> Generator[steps.Outline[object]]:
    constraint = cstr.combine(
        cstr.xyz(rover.L1, x=0, y=0, z=0),
        cstr.xyz(rover.L2, y=0, z=0),
        cstr.xyz(rover.R1, z=0),
        cstr.xyz(rover.R2, z=0),
    )
    while True:
        yield steps.Outline(constraint)


async def main(init_pos: Matrix = rover.CRAWLING_POS) -> None:
    robot = TrussController(
        truss=rover.RoverTruss.make_pos(init_pos),
        lin_cost=SingularityCost(),
    )
    animation = rover.set_up_animation(init_pos, anim.make_default_view())
    try:
        for outline in sit():
            step = robot.build_step(outline)
            dx = step.solve(robot.state)
            robot.nudge(dx)
            animation.update_pos(robot.truss.source.pos)
            await asyncio.sleep(0)
    except steps.InverseKinematicsError as e:
        print(e.args[0])
    print("Done with animation")


if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main())
