import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import serial

from rift.arraytypes import Vector
from . import commands, reception


@dataclass(slots=True)
class Commander:
    ser: serial.Serial
    log: Callable[[str], object] | None = None

    def send(self, cmd: bytes) -> None:
        self.ser.write(cmd)
        self.ser.flush()
        if self.log is not None:
            self.log("Sent: " + cmd.decode().strip())

    def send_stop(self) -> None:
        self.send(commands.STOP)

    def set_zero(self) -> None:
        self.send_stop()
        self.send(commands.RESET)

    def to_zero(self) -> None:
        self.send_stop()
        self.send(commands.POS(()))

    def send_dq(self, dq: Vector[np.intp], dt: float) -> None:
        cmd = commands.VEL(dq/dt, dt)
        self.send(cmd)

    def send_q(self, q: Vector[np.intp]) -> None:
        cmd = commands.POS(q)
        self.send(cmd)

    def read_last_q(self) -> Vector[np.intp] | None:
        buffer = self.ser.read_all()
        if buffer is None:
            q_cur = None
        else:
            lines = buffer.splitlines(keepends=True)
            q_cur = reception.read_q(lines, self.log)
        return q_cur

    def get_q(self, *, delay: float = 0.03) -> Vector[np.intp] | None:
        self.ser.reset_output_buffer()
        self.send_stop()
        time.sleep(delay)
        return self.read_last_q()
