from collections.abc import Callable, Iterable
from dataclasses import dataclass

import numpy as np

from .linalg import Matrix, MatrixStack, Vector
from .truss_config import Lock
from .tubetruss import Node


@dataclass(slots=True, frozen=True)
class Step:
    node: Node
    velocity: float | Vector | Callable[[Vector], Matrix]
    locks: Iterable[Lock] = ()


def make_motion_array(step: Step, shape: tuple[int, int], resolution: int = 10) -> MatrixStack:
    t = np.linspace(0., 1., resolution)
    motion = np.full((len(t), *shape), np.nan)
    if callable(step.velocity):
        v = step.velocity(t).T
    else:
        v = step.velocity
    motion[:, step.node, :] = v
    for n, i in step.locks:
        motion[:, n, i] = 0.
    return motion
