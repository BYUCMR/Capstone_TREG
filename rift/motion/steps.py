from collections.abc import Generator, Iterable
from typing import Protocol

import numpy as np

from rift.arraytypes import Matrix, Vector
from . import constraints as cstr, optimize


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


def singularity_eig(A: Matrix, b: Vector | None = None) -> tuple[float, Vector]:
    evals, evecs = np.linalg.eigh(A.T @ A)
    m, n = A.shape
    if b is not None:
        aug = np.concat((A, b.reshape(-1, 1)), axis=1)
        # Note that this takes about as much time as getting the eigenvalues;
        # it's better to develop minimal constraints by hand.
        m = np.linalg.matrix_rank(aug)
    i = max(0, n - m)
    return evals[i], evecs[:, i]


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
    e, v = singularity_eig(Ab[:, :-1], Ab[:, -1] if allow_redundant else None)
    if abs(e) <= 1e-3:
        raise SingularityError("Robot state is singular")
    Gh = (
        None if ineq_constraint is None
        else robot.resolve_constraint(ineq_constraint)
    )
    if Gh is not None or allow_redundant:
        vel = optimize.solve_qp(R=cost, Ab=Ab, Gh=Gh, solver='piqp')
    else:
        vel = optimize.solve_kkt(R=cost, Ab=Ab)
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
