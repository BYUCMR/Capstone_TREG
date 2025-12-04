from collections.abc import Callable

import numpy as np
import qpsolvers

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


def fill_substep(
    outline: Matrix, *,
    R: Matrix,
    f: Vector | None = None,
    A: Matrix | None = None,
    b: Vector | None = None,
    solver: str = 'piqp',
) -> Matrix | None:
    _, n = R.shape
    if f is None:
        f = np.zeros(n)
    elif len(f) != n:
        raise ValueError(f"Wrong shape for f: expected ({n}, [1]), got {f.shape}")
    if A is None:
        A = np.zeros((0, n))
    elif A.shape[1] != n:
        raise ValueError(f"Wrong shape for A: expected (_, {n}), got {A.shape}")
    if b is None:
        b = np.zeros(len(A))
    elif len(b) != len(A):
        raise ValueError(f"Wrong shape for b: expected ({len(A)}, [1]), got {b.shape}")

    H = R.T @ R
    A_move, b_move = make_move_constraint(outline)
    A = np.vstack([A, A_move])
    b = np.concat([b, b_move])
    substep = qpsolvers.solve_qp(P=H, q=f, A=A, b=b, solver=solver)
    if substep is None:
        return None
    else:
        return substep.reshape(outline.shape)
