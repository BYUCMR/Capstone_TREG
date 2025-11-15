from collections.abc import Callable, Iterable
from dataclasses import dataclass

from .linalg import Vector, unit_vector
from .truss_config import Lock


@dataclass(slots=True, frozen=True, kw_only=True)
class Step:
    node: int
    target: Vector
    tol: float = 0.01
    dt: float = 0.01
    locks: Iterable[Lock] = ()
    ctrl_func: Callable[[Vector], Vector] = unit_vector
