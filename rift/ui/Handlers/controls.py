from collections.abc import Generator, Iterable
from dataclasses import dataclass

from rift import rover
from rift import steps
from rift import tubetruss as tt
from rift.arraytypes import Vector


def take_command(
    robot: tt.TrussRobot,
    command: steps.Command,
) -> Generator[Vector]:
    if not command:
        return
    elif command.mode is steps.Mode.crawling:
        x = command.x * 0.125
        y = -command.y * 0.125
        yield from rover.crawl(robot, 1, (x, y), resolution=100)
    elif command.mode is steps.Mode.node_control:
        yield rover.nudge_node(
            robot,
            command.item,
            command.x * 0.0005,
            command.y * 0.0005,
            command.z * 0.0005,
        )
    elif command.mode is steps.Mode.calibration:
        yield rover.adjust_roller(robot, command.item, command.x * 0.0005)
    elif command.mode is steps.Mode.stand:
        yield rover.nudge_chassis(
            robot,
            0,
            0,
            command.z * 0.0005,
        )


@dataclass(slots=True)
class Bundler:
    period: int
    _i: int = 0

    def expend(self, gen: Iterable[Vector]) -> Generator[Vector]:
        delta_q = None
        for dq in gen:
            self._i += 1
            if delta_q is None:
                delta_q = dq
            else:
                delta_q += dq
            if not self._i % self.period:
                yield delta_q
                delta_q = None
