from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
import qpsolvers

from rift.arraytypes import Matrix, Vector
from . import constrain as cstr
from .control import LengthControl
from .linalg import get_rigidity


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


class RobotLike(Protocol):
    @property
    def dx_to_dq(self, /) -> Matrix: ...
    def nudge(self, dx: Vector, /) -> object: ...
    def resolve_constraint(self, constraint: cstr.Constraint, /) -> Matrix: ...


def apply_roll(
    robot: RobotLike,
    dq: Vector,
    constraint: cstr.Constraint,
) -> Matrix:
    Ab = robot.resolve_constraint(constraint)
    dx = np.linalg.solve(
        np.concat((robot.dx_to_dq, Ab[:, :-1])),
        np.concat((dq, Ab[:, -1])),
    )
    robot.nudge(dx)
    return dx


def solve_qp(
    *,
    R: Matrix,
    f: Vector | None = None,
    Ab: Matrix | None = None,
    Gh: Matrix | None = None,
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
        raise ValueError(f"Wrong shape for f: expected ({n}, [1]); got {f.shape}")
    if Ab is None:
        Ab = np.zeros((0, n+1))
    elif Ab.shape[1] != n+1:
        raise ValueError(f"Wrong shape for [A|b]: expected (_, {n+1}); got {Ab.shape}")
    if solver == 'kkt' and Gh is not None:
        raise ValueError("Cannot use KKT to solve with inequality constraints")
    if Gh is None:
        Gh = np.zeros((0, n+1))
    elif Gh.shape[1] != n+1:
        raise ValueError(f"Wrong shape for [G|h]: expected (_, {n+1}); got {Gh.shape}")

    A = Ab[:, :-1]
    G = Gh[:, :-1]
    b = Ab[:, -1]
    h = Gh[:, -1]
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


def take_step(
    robot: RobotLike,
    constraint: cstr.Constraint,
    ineq_constraint: cstr.Constraint | None = None,
    *,
    dt: float = 1.,
    cost: Matrix | None = None,
    allow_redundant: bool = False,
) -> Vector:
    if cost is None:
        cost = robot.dx_to_dq
    Ab = robot.resolve_constraint(constraint)
    e, v = cstr.singularity_eig(Ab[:, :-1], Ab[:, -1] if allow_redundant else None)
    if abs(e) <= 1e-3:
        raise SingularityError("Robot state is singular")
    if ineq_constraint is None:
        Gh = None
        solver = 'piqp' if allow_redundant else 'kkt'
    else:
        Gh = robot.resolve_constraint(ineq_constraint)
        solver = 'piqp'
    vel = solve_qp(R=cost, Ab=Ab, Gh=Gh, solver=solver)
    if vel is None:
        raise SolverError("Could not find valid node velocities")
    dx = vel * dt
    dq = robot.dx_to_dq @ dx
    robot.nudge(dx)
    return dq


def divide_steps(
    robot: RobotLike,
    steps: Iterable[cstr.Constraint],
    *,
    resolution: int,
    allow_redundant: bool = False,
) -> Generator[Vector]:
    dt = 1 / resolution
    for step in steps:
        for _ in range(resolution):
            yield take_step(
                robot,
                step,
                dt=dt,
                allow_redundant=allow_redundant,
            )


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

    @property
    def dx_to_dq(self) -> Matrix:
        return self.control.inverse @ self.rigidity

    def resolve_constraint(self, constraint: cstr.Constraint) -> Matrix:
        length_constraint = cstr.Static.make_hom(self.control.unreachable @ self.rigidity)
        constraint = cstr.CompoundConstraint((length_constraint, constraint))
        return constraint.at(self.pos)

    def nudge(self, dx: Matrix | Vector) -> None:
        if len(dx.shape) == 1:
            dx = dx.reshape(self._pos.shape)
        self._rigidity = None
        self._pos[:] += dx

    apply_roll = apply_roll
    take_step = take_step
    divide_steps = divide_steps
