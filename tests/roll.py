import asyncio

import pyqtgraph
from PySide6 import QtAsyncio

from rift import rover
from rift.arraytypes import Matrix
from rift.motion import InverseKinematicsError


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
        for i in range(3):
            for _ in robot.divide_steps(rover.roll(), resolution=resolution):
                stabilizer.update_pos(robot.source.pos)
                animate(stabilizer.pos)
                await asyncio.sleep(0)
            robot.permuter @= rover.ROLL
    except InverseKinematicsError as e:
        print(e.args[0])
    print("Done with animation")


if __name__ == '__main__':
    pyqtgraph.mkQApp()
    QtAsyncio.run(main())
