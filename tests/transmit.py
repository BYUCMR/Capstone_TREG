import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import asyncio
import io

import pyqtgraph
import serial
from PySide6 import QtAsyncio

from rift import rover
from rift.arraytypes import Matrix
from rift.robot import InverseKinematicsError
from rift.transmit import commands


async def main(
    ser: io.IOBase | None = None,
    init_pos: Matrix = rover.CRAWLING_POS,
    *,
    resolution: int = 100,
) -> None:
    view, animate = rover.set_up_animation(init_pos)
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    view.show()
    t = 37.5 / resolution
    try:
        for dr in rover.roll(robot, resolution=resolution):
            stabilizer.update_pos(robot.pos)
            animate(stabilizer.pos)
            ticks_per_sec = rover.TICKS_PER_SIDE * dr / t
            cmd = commands.VEL(map(int, ticks_per_sec), t)
            if ser is not None:
                ser.writelines((commands.STOP, cmd))
                ser.flush()
                print("[SENT]", cmd.decode(), end="")
            await asyncio.sleep(t)
    except InverseKinematicsError as e:
        print(e.args[0])


if __name__ == "__main__":
    init_pos = rover.ROLLING_POS
    SERIAL_PORT = '/dev/ttyUSB0'
    BAUD_RATE = 115200
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException:
        ser = open("commands.txt", 'wb')
    else:
        print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
    pyqtgraph.mkQApp()
    try:
        QtAsyncio.run(main(ser, init_pos, resolution=25))
    finally:
        if ser is not None:
            ser.write(commands.STOP)
            ser.flush()
            ser.close()
            print(f"Serial port {SERIAL_PORT} closed.")
