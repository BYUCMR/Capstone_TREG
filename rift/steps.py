from dataclasses import dataclass
from enum import Enum

import numpy as np
import qpsolvers

from .arraytypes import Matrix, Vector


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


def parabolic(k: float, t: float) -> float:
    return 2. * k * (0.5-t)


def find_dx(
    *,
    R: Matrix,
    f: Vector | None = None,
    A: Matrix | None = None,
    b: Vector | None = None,
) -> Vector | None:
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
    m, n = A.shape
    O = np.zeros((m, m))
    K = np.vstack([np.hstack([H, A.T]), np.hstack([A, O])])
    try:
        x_l = np.linalg.solve(K, np.concat([-f, b]))
    except np.linalg.LinAlgError:
        return None
    # `x` is equivalent to what we'd get from solving a quadratic program
    # minimizing `x'*H*x + f'*x` subject to `A*x = b`.
    x, l = np.split(x_l, [n])
    return x
