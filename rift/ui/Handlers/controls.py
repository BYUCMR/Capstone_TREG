from collections.abc import Generator, Iterable
from dataclasses import dataclass
from enum import Enum
from typing import SupportsIndex

import numpy as np

from rift import rover
from rift import tubetruss as tt
from rift.arraytypes import Vector


class Mode(Enum):
    crawling = "crawling"
    offline = "offline"
    node_control = "node_control"
    calibration = "calibration"
    stand = "stand"
    rolling = "rolling"


@dataclass
class Command:
    mode: Mode
    item: SupportsIndex
    x: float
    y: float
    z: float

    def __bool__(self) -> bool:
        return self.mode is not Mode.offline and bool(self.x or self.y or self.z)


def take_command(
    robot: tt.TrussRobot,
    command: Command,
) -> Generator[Vector]:
    if not command:
        return
    elif command.mode is Mode.crawling:
        x = command.x * 0.125
        y = command.y * 0.125
        if not (x or y):
            return
        yield from robot.divide_steps(rover.crawl(1, (x, y)), resolution=100)
    elif command.mode is Mode.node_control:
        motion = rover.node_nudge(
            command.item,
            command.x * 0.0005,
            command.y * 0.0005,
            command.z * 0.0005,
        )
        yield robot.take_step(motion, respect_floor=True, allow_redundant=True)
    elif command.mode is Mode.calibration:
        yield rover.adjust_roller(robot, command.item, command.x * 0.0005)
    elif command.mode is Mode.stand:
        motion = rover.chassis_nudge(
            command.x * 0.0005,
            command.y * 0.0005,
            command.z * 0.0005,
        )
        yield robot.take_step(motion)
    elif command.mode is Mode.rolling:
        motion = rover.chassis_tilt(np.pi * command.x / 1000)
        yield robot.take_step(motion)


@dataclass(slots=True)
class Bundler:
    period: int
    _i: int = 0
    delta_q: Vector | None = None

    def expend(self, gen: Iterable[Vector]) -> Generator[Vector]:
        for dq in gen:
            self._i += 1
            if self.delta_q is None:
                self.delta_q = dq
            else:
                self.delta_q += dq
            if not self._i % self.period:
                yield self.delta_q
                self.delta_q = None
