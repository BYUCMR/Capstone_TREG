from dataclasses import dataclass

from .typing import Matrix, Vector


@dataclass(slots=True, kw_only=True, frozen=True)
class RobotState:
    pos: Matrix
    roll: Vector
