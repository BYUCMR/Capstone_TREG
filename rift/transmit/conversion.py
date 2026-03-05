import io
from collections.abc import Iterable
from typing import SupportsInt

import numpy as np

from rift.arraytypes import Matrix

from . import commands as commands


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
