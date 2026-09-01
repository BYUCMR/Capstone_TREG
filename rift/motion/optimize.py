import numpy as np
import qpsolvers

from rift.arraytypes import Matrix, Vector


def solve_kkt(
    *,
    R: Matrix,
    f: Vector | None = None,
    Ab: Matrix | None = None,
) -> Vector | None:
    """
    Find `x` such that `x'*R'*R*x + f'*x` is minimized and `A*x = b`.

    Return `None` if no such `x` exists.

    `R` must be specificed, and all other inputs default to zeros of the
    correct shapes.
    """
    _, n = R.shape
    if f is None:
        f = np.zeros(n)
    elif len(f) != n:
        raise ValueError(f"Wrong shape for f: expected ({n}, [1]), got {f.shape}")
    if Ab is None:
        Ab = np.zeros((0, n+1))
    elif Ab.shape[1] != n+1:
        raise ValueError(f"Wrong shape for [A|b]: expected (_, {n+1}), got {Ab.shape}")

    A = Ab[:, :-1]
    b = Ab[:, -1]
    H = R.T @ R
    m, n = A.shape
    O = np.zeros((m, m))
    K = np.concat((np.concat((H, A.T), axis=1), np.concat((A, O), axis=1)))
    try:
        x_l = np.linalg.solve(K, np.concat((-f, b)))
    except np.linalg.LinAlgError:
        return None
    x, l = np.split(x_l, (n,))
    return x


def solve_qp(
    *,
    R: Matrix,
    f: Vector | None = None,
    Ab: Matrix | None = None,
    Gh: Matrix | None = None,
    solver: str = 'piqp',
) -> Vector | None:
    """
    Find `x` such that `x'*R'*R*x + f'*x` is minimized, `A*x = b`, and `G*x <= h`.

    Return `None` if no such `x` exists.

    `R` must be specificed, and all other inputs default to zeros of the
    correct shapes.
    """
    _, n = R.shape
    if f is None:
        f = np.zeros(n)
    elif len(f) != n:
        raise ValueError(f"Wrong shape for f: expected ({n}, [1]), got {f.shape}")
    if Ab is None:
        Ab = np.zeros((0, n+1))
    elif Ab.shape[1] != n+1:
        raise ValueError(f"Wrong shape for [A|b]: expected (_, {n+1}), got {Ab.shape}")
    if Gh is None:
        Gh = np.zeros((0, n+1))
    elif Gh.shape[1] != n+1:
        raise ValueError(f"Wrong shape for [G|h]: expected (_, {n+1}), got {Gh.shape}")
    A = Ab[:, :-1]
    G = Gh[:, :-1]
    b = Ab[:, -1]
    h = Gh[:, -1]
    H = R.T @ R
    return qpsolvers.solve_qp(P=H, q=f, A=A, b=b, G=G, h=h, solver=solver)
