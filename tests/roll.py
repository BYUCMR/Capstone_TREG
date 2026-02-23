import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import asyncio

import pyqtgraph
from PySide6 import QtAsyncio

from rift import rover
from rift.arraytypes import Matrix
from rift.robot import InverseKinematicsError


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
        for _ in rover.roll(robot, resolution=resolution):
            stabilizer.update_pos(robot.pos)
            await positions.put(stabilizer.pos)

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
