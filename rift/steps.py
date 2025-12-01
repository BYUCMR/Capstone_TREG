from collections.abc import Callable

import numpy as np

from .tubetruss import Node
from .typing import Matrix, MatrixStack, Vector


type StepVelocity = float | Vector | Callable[[Vector], Matrix]


def parabola(t: Vector, d: float = 1.0) -> Matrix:
    k = d / len(t)
    u = np.full_like(t, k)
    v = np.zeros_like(t)
    w = 2. * k * (0.5-t)
    return np.column_stack([u, v, w])


def make_step_array(
    shape: tuple[int, int],
    *commands: *tuple[tuple[Node, StepVelocity], ...],
    resolution: int = 10,
) -> MatrixStack:
    t = np.linspace(0., 1., resolution)
    motion = np.full((resolution, *shape), np.nan)
    for node, velocity in commands:
        if callable(velocity):
            v = velocity(t)
        else:
            v = velocity
        motion[:, node, :] = v
    return motion


def make_move_constraint(substep: Matrix) -> tuple[Matrix, Vector]:
    i = ~np.isnan(substep)
    b = substep[i]
    A = np.zeros((b.size, i.size))
    A[:, i.flat] = np.eye(b.size)
    return A, b
