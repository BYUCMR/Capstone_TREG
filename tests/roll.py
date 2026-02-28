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
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    view, animate = rover.set_up_animation(init_pos)
    view.show()
    try:
        for _ in rover.roll(robot, resolution=resolution):
            stabilizer.update_pos(robot.pos)
            animate(stabilizer.pos)
            await asyncio.sleep(0.01)
    except InverseKinematicsError as e:
        print(e.args[0])
    print("Done with animation")


if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main())
