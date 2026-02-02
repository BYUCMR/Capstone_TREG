from dataclasses import dataclass
from enum import Enum


class Mode(Enum):
    crawling = "crawling"
    offline = "offline"
    node_control = "node_control"


@dataclass
class Command:
    mode: Mode
    x: float
    y: float
    z: float
