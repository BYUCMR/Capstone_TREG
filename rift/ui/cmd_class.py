from dataclasses import dataclass
from enum import Enum


class Mode(Enum):
    crawling = "crawling"
    offline = "offline"
    node_control = "node_control"
    calibration = "calibration"


@dataclass
class Command:
    mode: Mode
    item: int
    x: float
    y: float
    z: float
