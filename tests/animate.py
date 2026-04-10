import asyncio

import pyqtgraph
from PySide6 import QtAsyncio

from rift import rover
from rift.arraytypes import Matrix
from rift.motion import InverseKinematicsError


async def main(
    init_pos: Matrix = rover.CRAWLING_POS,
    *,
    step_length: float = 0.125,
    cycles: int = 1,
    resolution: int = 50,
) -> None:
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    view, animate = rover.set_up_animation(init_pos)
    view.show()
    try:
        steps = rover.crawl(cycles, (step_length, 0))
        for _ in robot.divide_steps(steps, resolution=resolution):
            stabilizer.update_pos(robot.pos)
            animate(stabilizer.pos)
            await asyncio.sleep(0)
    except InverseKinematicsError as e:
        print(e.args[0])
    print("Done with animation")


if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main(cycles=4, resolution=100))
