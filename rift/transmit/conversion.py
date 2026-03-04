import io
from collections.abc import Iterable
from typing import SupportsInt

import numpy as np

from rift.arraytypes import Matrix, Vector

from . import commands as commands


def dist_to_ticks(perimeter: float, command: Vector):
    ticks_per_foot = 1125 * 12
    return np.array(command) * perimeter * ticks_per_foot


def ticks_to_tps(command: Vector, t: float = 1.5):
    return command / t


def send_command(ser: io.IOBase, v: Iterable[SupportsInt], t: float) -> None:
    commands.send(ser, commands.VEL(map(int, v), t))


def send_command_pos(ser: io.IOBase, q: Iterable[SupportsInt]) -> None:
    commands.send(ser, commands.VEL(map(int, q)))


def send_stop(ser: io.IOBase) -> None:
    commands.send(ser, commands.STOP())


def read_positions(ser: io.IOBase) -> Matrix | None:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if line.startswith('[') and line.endswith(']'):
            try:
                values = np.fromstring(line[1:-1], sep=',')
                return -values.reshape(-1, 1)
            except Exception:
                print(f"[WARN] Could not parse: {line}")
                return None
        else:
            # Print other serial messages like warnings
            print(f"[OTHER] {line}")
            return
