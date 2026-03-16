import io
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from rift.arraytypes import Vector
from . import commands, reception


@dataclass(slots=True, kw_only=True)
class Commander:
    ser: io.IOBase
    q_des: Vector[np.intp]
    log: Callable[[str], object] | None = None

    def update(self, dq: Vector[np.intp]) -> None:
        self.q_des += dq

    def set_zero(self) -> None:
        self.send(commands.STOP)
        self.send(commands.RESET)
        self.q_des[:] = 0

    def to_zero(self) -> None:
        self.q_des[:] = 0
        self.send(commands.STOP)
        self.send(commands.POS(()))

    def send(self, cmd: bytes) -> None:
        self.ser.write(cmd)
        self.ser.flush()
        if self.log is not None:
            self.log("Sent: " + cmd.decode().strip())

    def send_dq(self, dq: Vector[np.intp], dt: float) -> None:
        cmd = commands.VEL(dq, dt)
        self.send(cmd)
        time.sleep(dt)

    def get_error(self) -> Vector[np.intp] | None:
        self.send(commands.STOP)
        time.sleep(0.03)
        lines = self.ser.readlines()
        q_cur = reception.read_q(lines, self.log)
        if q_cur is None:
            return None
        else:
            return self.q_des - q_cur

    def catch_up(self) -> None:
        self.send(commands.STOP)
        self.send(commands.POS(self.q_des))
