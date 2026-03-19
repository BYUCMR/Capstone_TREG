from collections.abc import Generator
from dataclasses import dataclass

import numpy as np
import qpsolvers

from rift.arraytypes import Matrix, Vector
from . import constrain as cstr
from .control import LengthControl
from .trusses import Truss


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


def find_dx(
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


@dataclass(slots=True)
class TrussRobot:
    """
    A representation of a truss robot.

    It comprises a position, a truss structure, and a control setup.
    """
    pos: Matrix
    truss: Truss
    control: LengthControl

    def __post_init__(self) -> None:
        if len(self.pos) != self.truss.n_nodes:
            raise ValueError("Robot position and truss have mismatched node counts")
        if self.truss.n_links != self.control.n_outputs:
            raise ValueError("Robot truss and control have mismatched link counts")

    @property
    def n_nodes(self) -> int:
        return self.truss.n_nodes

    @property
    def n_rollers(self) -> int:
        return self.control.n_inputs

    def apply_roll(
        self,
        d_roll: Vector,
        *constraints: cstr.Constraint,
        t: float = 0.,
    ) -> Matrix:
        rigidity = self.truss.rigidity_at(self.pos)
        constraint = cstr.CompoundConstraint(constraints)
        A, b = constraint.get(self.pos, t)
        d_length = self.control.forward @ d_roll
        d_pos = np.linalg.solve(
            np.concat((rigidity, A)),
            np.concat((d_length, b)),
        )
        d_pos = d_pos.reshape(self.pos.shape)
        self.pos += d_pos
        return d_pos

    def take_substep(
        self,
        *constraints: cstr.Constraint,
        t: float = 0.,
        allow_redundant: bool = False,
        respect_floor: bool = False,
    ) -> Vector:
        rigidity = self.truss.rigidity_at(self.pos)
        constraint = cstr.CompoundConstraint((
            cstr.CustomConstraint(self.control.unreachable @ rigidity),
            *constraints
        ))
        A, b = constraint.get(self.pos, t)
        e, v = cstr.singularity_eig(A, b if allow_redundant else None)
        if abs(e) <= 1e-3:
            raise SingularityError("Robot state is singular")
        if respect_floor:
            G = np.zeros((self.n_nodes, self.pos.size))
            G[range(self.n_nodes), range(2, self.pos.size, 3)] = -1.
            h = self.pos[:,2]
            solver = 'piqp'
        else:
            G = None
            h = None
            solver = 'piqp' if allow_redundant else 'kkt'
        dx = find_dx(R=rigidity, A=A, b=b, G=G, h=h, solver=solver)
        if dx is None:
            raise SolverError("Could not find valid node velocities")
        dq = self.control.inverse @ rigidity @ dx
        self.pos += dx.reshape(self.pos.shape)
        return dq

    def take_step(
        self,
        *constraints: cstr.Constraint,
        resolution: int,
        allow_redundant: bool = False,
        respect_floor: bool = False,
    ) -> Generator[Vector]:
        for t in np.linspace(0., 1., resolution):
            yield self.take_substep(
                *constraints,
                t=t,
                allow_redundant=allow_redundant,
                respect_floor=respect_floor,
            )
