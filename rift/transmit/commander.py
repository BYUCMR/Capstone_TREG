import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import serial

from rift.arraytypes import Vector
from . import commands, reception


@dataclass(slots=True, kw_only=True)
class Commander:
    ser: serial.Serial
    q_des: Vector[np.intp]
    log: Callable[[str], object] | None = None

    def update(self, dq: Vector[np.intp]) -> None:
        self.q_des += dq

    def send(self, cmd: bytes) -> None:
        self.ser.write(cmd)
        self.ser.flush()
        if self.log is not None:
            self.log("Sent: " + cmd.decode().strip())

    def stop(self) -> None:
        self.send(commands.STOP)

    def set_zero(self) -> None:
        self.stop()
        self.send(commands.RESET)
        self.q_des[:] = 0

    def to_zero(self) -> None:
        self.q_des[:] = 0
        self.stop()
        self.send(commands.POS(()))

    def send_dq(self, dq: Vector[np.intp], dt: float) -> None:
        cmd = commands.VEL(dq, dt)
        self.send(cmd)
        time.sleep(dt)

    def get_error(self) -> Vector[np.intp] | None:
        self.ser.reset_output_buffer()
        self.stop()
        time.sleep(0.03)
        buffer = self.ser.read_all()
        if buffer is None:
            q_cur = None
        else:
            lines = buffer.splitlines(keepends=True)
            q_cur = reception.read_q(lines, self.log)
        if q_cur is None:
            return None
        else:
            return self.q_des - q_cur

    def catch_up(self) -> None:
        self.stop()
        self.send(commands.POS(self.q_des))
