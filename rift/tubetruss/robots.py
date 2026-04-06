from collections.abc import Generator, Iterable
from dataclasses import dataclass, field

import numpy as np
import qpsolvers

from rift.arraytypes import Matrix, Vector
from . import constrain as cstr
from .control import LengthControl
from .linalg import get_rigidity


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


def solve_qp(
    *,
    R: Matrix,
    f: Vector | None = None,
    A: Matrix | None = None,
    b: Vector | None = None,
    G: Matrix | None = None,
    h: Vector | None = None,
    solver: str = 'kkt',
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
    if A is None:
        A = np.zeros((0, n))
    elif A.shape[1] != n:
        raise ValueError(f"Wrong shape for A: expected (_, {n}), got {A.shape}")
    if b is None:
        b = np.zeros(len(A))
    elif len(b) != len(A):
        raise ValueError(f"Wrong shape for b: expected ({len(A)}, [1]), got {b.shape}")
    if solver == 'kkt' and (G is not None or h is not None):
        raise ValueError("Cannot use KKT to solve with inequality constraints")
    if G is None:
        G = np.zeros((0, n))
    elif G.shape[1] != n:
        raise ValueError(f"Wrong shape for G: expected (_, {n}), got {G.shape}")
    if h is None:
        h = np.zeros(len(G))
    elif len(h) != len(G):
        raise ValueError(f"Wrong shape for b: expected ({len(G)}, [1]), got {h.shape}")

    H = R.T @ R
    if solver != 'kkt':
        return qpsolvers.solve_qp(P=H, q=f, A=A, b=b, G=G, h=h, solver=solver)

    m, n = A.shape
    O = np.zeros((m, m))
    K = np.concat((np.concat((H, A.T), axis=1), np.concat((A, O), axis=1)))
    try:
        x_l = np.linalg.solve(K, np.concat((-f, b)))
    except np.linalg.LinAlgError:
        return None
    x, l = np.split(x_l, [n])
    return x


def find_dx(
    *,
    x: Matrix,
    cost: Matrix,
    constraint: cstr.Constraint,
    scale: float = 1,
    allow_redundant: bool = False,
    respect_floor: bool = False,
) -> Matrix:
    A, b = constraint.get(x, scale)
    e, v = cstr.singularity_eig(A, b if allow_redundant else None)
    if abs(e) <= 1e-3:
        raise SingularityError("Robot state is singular")
    if respect_floor:
        G = np.zeros((x.shape[0], x.size))
        G[range(x.shape[0]), range(2, x.size, 3)] = -1.
        h = x[:,2]
        solver = 'piqp'
    else:
        G = None
        h = None
        solver = 'piqp' if allow_redundant else 'kkt'
    dx = solve_qp(R=cost, A=A, b=b, G=G, h=h, solver=solver)
    if dx is None:
        raise SolverError("Could not find valid node velocities")
    return dx.reshape(x.shape)


@dataclass(slots=True)
class TrussRobot:
    """
    A representation of a truss robot.

    It comprises a position, a truss structure, and a control setup.
    """
    _pos: Matrix
    _incidence: Matrix[np.int8]
    control: LengthControl
    _rigidity: Matrix | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self._pos.shape[0] != self._incidence.shape[1]:
            raise ValueError("Robot position and incidence have mismatched node counts")
        if self._incidence.shape[0] != self.control.n_outputs:
            raise ValueError("Robot incidence and control have mismatched link counts")

    @property
    def n_nodes(self) -> int:
        return len(self._pos)

    @property
    def n_rollers(self) -> int:
        return self.control.n_inputs

    @property
    def pos(self) -> Matrix:
        view = self._pos.view()
        view.setflags(write=False)
        return view

    @property
    def incidence(self) -> Matrix[np.int8]:
        view = self._incidence.view()
        view.setflags(write=False)
        return view

    @property
    def rigidity(self) -> Matrix:
        if self._rigidity is None:
            self._rigidity = get_rigidity(self._incidence, self._pos)
        return self._rigidity

    def apply_roll(
        self,
        dq: Vector,
        constraint: cstr.Constraint,
    ) -> Matrix:
        A, b = constraint.get(self.pos, 1.)
        dL = self.control.forward @ dq
        dx = np.linalg.solve(
            np.concat((self.rigidity, A)),
            np.concat((dL, b)),
        )
        dx = dx.reshape(self.pos.shape)
        self._rigidity = None
        self._pos[:] += dx
        return dx

    def take_step(
        self,
        *constraints: cstr.Constraint,
        scale: float = 1,
        allow_redundant: bool = False,
        respect_floor: bool = False,
    ) -> Vector:
        constraint = cstr.CompoundConstraint((
            cstr.Static(self.control.unreachable @ self.rigidity),
            *constraints
        ))
        dx = find_dx(
            x=self.pos,
            cost=self.rigidity,
            constraint=constraint,
            scale=scale,
            allow_redundant=allow_redundant,
            respect_floor=respect_floor,
        )
        dq = self.control.inverse @ self.rigidity @ dx.ravel()
        self._rigidity = None
        self._pos[:] += dx
        return dq

    def divide_steps(
        self,
        steps: Iterable[cstr.Constraint],
        *,
        resolution: int,
        allow_redundant: bool = False,
        respect_floor: bool = False,
    ) -> Generator[Vector]:
        scale = 1 / resolution
        for step in steps:
            for _ in range(resolution):
                yield self.take_step(
                    step,
                    scale=scale,
                    allow_redundant=allow_redundant,
                    respect_floor=respect_floor,
                )
