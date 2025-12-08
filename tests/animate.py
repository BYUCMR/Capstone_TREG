import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import asyncio

import pyqtgraph
from PySide6 import QtAsyncio

import rift.anim
from rift.robot import InverseKinematicsError, RobotInverse
from rift.truss_config import TrussConfig
from rift.typing import Matrix


async def main(
    config: TrussConfig,
    *,
    step_length: float = 0.8,
    cycles: int = 1,
    resolution: int = 50,
) -> None:
    animator = rift.anim.Animator.from_config(config)
    robot = RobotInverse.from_config(config)
    positions = asyncio.Queue[Matrix](resolution)

    async def crawl() -> None:
        for _ in robot.crawl(cycles, step_length, resolution=resolution):
            await positions.put(robot.pos.copy())

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
    from rift.truss_config import CONFIG_ROVER as config
    pyqtgraph.mkQApp()
    QtAsyncio.run(main(config, cycles=4, resolution=100))
