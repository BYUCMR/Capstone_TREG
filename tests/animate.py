import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import asyncio

import pyqtgraph
from PySide6 import QtAsyncio

import rift.anim
import rift.grav
from rift.arraytypes import Matrix
from rift.robot import InverseKinematicsError, RobotInverse
from rift.truss_config import TrussConfig


async def main(
    config: TrussConfig,
    *,
    step_length: float = 0.125,
    cycles: int = 1,
    resolution: int = 50,
) -> None:
    animator = rift.anim.Animator.from_config(config)
    robot = RobotInverse.from_config(config)
    stabilizer = rift.grav.Stabilizer.from_config(config)
    positions = asyncio.Queue[Matrix](resolution)

    async def crawl() -> None:
        for _ in robot.crawl(cycles, step_length, resolution=resolution):
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
    from rift.truss_config import ROVER_CONFIG as config
    pyqtgraph.mkQApp()
    QtAsyncio.run(main(config, cycles=4, resolution=100))
