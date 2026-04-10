from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
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


class Step[RobotType](Protocol):
    def find_vel(self, robot: RobotType, /) -> Vector: ...


class Robot(Protocol):
    @property
    def dx_to_dq(self, /) -> Matrix: ...
    def resolve_constraint(self, constraint: cstr.Constraint, /) -> Matrix: ...


class MovableRobot(Robot, Protocol):
    def nudge(self, dx: Vector, /) -> object: ...


@dataclass(slots=True)
class FKStep(Step[Robot]):
    qdot: Vector
    constraint: cstr.Constraint

    def find_vel(self, robot: Robot) -> Vector:
        Ab = robot.resolve_constraint(self.constraint)
        return np.linalg.solve(
            np.concat((robot.dx_to_dq, Ab[:, :-1])),
            np.concat((self.qdot, Ab[:, -1])),
        )


@dataclass(slots=True)
class KKTStep(Step[Robot]):
    constraint: cstr.Constraint
    quad_cost: Matrix | None = None
    lin_cost: Vector | None = None

    def find_vel(self, robot: Robot) -> Vector:
        cost = robot.dx_to_dq if self.quad_cost is None else self.quad_cost
        Ab = robot.resolve_constraint(self.constraint)
        e, v = singularity_eig(Ab[:, :-1])
        if abs(e) <= 1e-3:
            raise SingularityError("Robot state is singular")
        vel = optimize.solve_kkt(R=cost, f=self.lin_cost, Ab=Ab)
        if vel is None:
            raise SolverError("Could not find valid node velocities")
        return vel


@dataclass(slots=True)
class QPStep(Step[Robot]):
    eq_constraint: cstr.Constraint
    le_constraint: cstr.Constraint | None = None
    quad_cost: Matrix | None = None
    lin_cost: Vector | None = None
    allow_redundancy: bool = field(default=False, kw_only=True)

    def find_vel(self, robot: Robot) -> Vector:
        cost = robot.dx_to_dq if self.quad_cost is None else self.quad_cost
        Ab = robot.resolve_constraint(self.eq_constraint)
        e, v = singularity_eig(Ab[:, :-1], Ab[:, -1] if self.allow_redundancy else None)
        if abs(e) <= 1e-3:
            raise SingularityError("Robot state is singular")
        Gh = (
            None if self.le_constraint is None
            else robot.resolve_constraint(self.le_constraint)
        )
        vel = optimize.solve_qp(R=cost, f=self.lin_cost, Ab=Ab, Gh=Gh, solver='piqp')
        if vel is None:
            raise SolverError("Could not find valid node velocities")
        return vel


def take_step[R: MovableRobot](robot: R, step: Step[R], *, dt: float = 1.) -> Vector:
    vel = step.find_vel(robot)
    dx = vel * dt
    dq = robot.dx_to_dq @ dx
    robot.nudge(dx)
    return dq


def divide_steps[R: MovableRobot](
    robot: R,
    steps: Iterable[Step[R]],
    *,
    resolution: int,
) -> Generator[Vector]:
    dt = 1 / resolution
    for step in steps:
        for _ in range(resolution):
            yield take_step(robot, step, dt=dt)
