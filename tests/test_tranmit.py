import pathlib, sys
sys.path.append(str(pathlib.Path.cwd()))

import asyncio
from collections.abc import Generator

import numpy as np
import pyqtgraph
import serial
from PySide6 import QtAsyncio

import rift.constrain as cstr
from rift import rover
from rift.arraytypes import Matrix, Vector
from rift.robot import InverseKinematicsError, RobotInverse
from rift.transmit.conversion import *
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200


def roll(
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


async def animate(
    init_pos: Matrix = rover.CRAWLING_POS,
    *,
    rollqueue: asyncio.Queue[Vector] | None = None,
    motion: cstr.Constraint,
    resolution: int = 50,
) -> None:
    animator = rover.make_animator(init_pos)
    robot = rover.make_robot(init_pos)
    stabilizer = rover.make_stabilizer(init_pos)
    positions = asyncio.Queue[Matrix](resolution)

    async def move() -> None:
        dq = np.zeros(12)
        for i, (*_, dr) in enumerate(rover.roll(robot, resolution=resolution)):
            stabilizer.update_pos(robot.pos)
            await positions.put(stabilizer.pos)
            if rollqueue is not None:
                await rollqueue.put(dr)
            # dq += dr
            # if i >= 5:
            #     break
        # if rollqueue is not None:
        #     await rollqueue.put(dq)
            

    crawling_task = asyncio.create_task(move())
    animation_task = asyncio.create_task(animator.animate(positions))
    try:
        await crawling_task
    except InverseKinematicsError as e:
        print(e.args[0])
    print("Done with IK")
    positions.shutdown()
    await animation_task
    print("Done with animation")


async def command(rollqueue: asyncio.Queue[Vector],resolution) -> None:
    async def cmd() -> None:
        t=1.5/resolution*25
        while True:
            try:
                d_roll = await rollqueue.get()
            except asyncio.QueueShutDown:
                break
            # print(d_roll)
            cmnd = ticks_to_tps(dist_to_ticks(6,d_roll))
            send_stop(ser)
            message = send_command(ser,cmnd,t)
            print(message)

    crawling_task = asyncio.create_task(cmd())
    await crawling_task
    print("Done sending commands")


async def main(
    init_pos: Matrix = rover.CRAWLING_POS,
    *,
    resolution: int = 50,
) -> None:
    payload_mass = np.zeros(12)
    payload_mass[rover.PAYLOAD] = 1
    motion = cstr.CompoundConstraint([
        cstr.Motion.make(rover.CR1, z=-1/resolution/10),
        cstr.Motion.lock(rover.CR2),
        cstr.Motion.lock(rover.CL1),
        cstr.Motion.lock(rover.CL2),
        # cstr.Motion.lock(rover.CPR3),
        # cstr.Motion.lock(rover.CPL3),
        # cstr.Motion.make(cstr.Point.com(payload_mass), x=0),
    ])
    rollqueue = asyncio.Queue[Vector]()
    animation_task = asyncio.create_task(animate(
        init_pos,
        rollqueue=rollqueue,
        motion=motion,
        resolution=resolution,
    ))
    command_task = asyncio.create_task(command(rollqueue,resolution))
    await animation_task
    await command_task


if __name__ == "__main__":
    config = rover.setup_rover_builder(1)
    try:
        ser = serial.Serial(SERIAL_PORT,BAUD_RATE,timeout=1)
        print(f"Serial port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")

        time.sleep(2)
        pyqtgraph.mkQApp()
        QtAsyncio.run(main(config, resolution=25))

    except serial.SerialException as e:
        print(f"Error opening or communicating with serial port: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        send_stop(ser)
        # Close the serial port
        if 'ser' in locals() and ser.isOpen():
            ser.close()
            print(f"Serial port {SERIAL_PORT} closed.")
