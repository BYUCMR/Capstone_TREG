from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from typing import Protocol, Self

import numpy as np

from rift.arraytypes import Matrix, Vector
from rift.protocols import StateFunction
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


@dataclass(slots=True, frozen=True)
class Outline[T]:
    eq: cstr.Constraint[T]
    le: cstr.Constraint[T] | None = None
    allow_redundancy: bool = field(default=False, kw_only=True)

    def expand(
        self,
        *,
        eq: cstr.Constraint[T] | None = None,
        le: cstr.Constraint[T] | None = None,
        allow_redundancy: bool = False,
    ) -> Self:
        if eq is None:
            eq = self.eq
        else:
            eq = cstr.combine(self.eq, eq)
        if le is None:
            le = self.le
        elif self.le is not None:
            le = cstr.combine(self.le, le)
        allow_redundancy = self.allow_redundancy or allow_redundancy
        return type(self)(eq, le, allow_redundancy=allow_redundancy)

    def __and__(self, other: Self) -> Self:
        return self.expand(eq=other.eq, le=other.le, allow_redundancy=other.allow_redundancy)


class Step[T](Protocol):
    def solve(self, state: T, /) -> Vector: ...


class MovableRobot(Protocol):
    def build_step(self, outline: Outline[Self], /) -> Step[Self]: ...
    def nudge(self, change: Vector, /) -> Vector: ...


@dataclass(slots=True)
class QPStep[T](Step[T]):
    quad_cost: StateFunction[T, Matrix]
    lin_cost: StateFunction[T, Vector] | None
    outline: Outline[T]

    def solve(self, state: T) -> Vector:
        Ab = self.outline.eq.at(state)
        e, v = singularity_eig(Ab[:, :-1], Ab[:, -1] if self.outline.allow_redundancy else None)
        if abs(e) <= 1e-3:
            raise SingularityError("Robot state is singular")
        Gh = (
            None if self.outline.le is None
            else self.outline.le.at(state)
        )
        R = self.quad_cost.at(state)
        f = None if self.lin_cost is None else self.lin_cost.at(state)
        if Gh is not None or self.outline.allow_redundancy:
            vel = optimize.solve_qp(R=R, f=f, Ab=Ab, Gh=Gh, solver='piqp')
        elif Ab.shape[1] == Ab.shape[0]+1:
            try:
                vel = np.linalg.solve(Ab[:, :-1], Ab[:, -1])
            except np.linalg.LinAlgError:
                vel = None
        else:
            vel = optimize.solve_kkt(R=R, f=f, Ab=Ab)
        if vel is None:
            raise SolverError("Could not find valid node velocities")
        return vel


def take_step[R: MovableRobot](
    robot: R,
    outline: Outline[R],
    *,
    dt: float = 1.,
) -> Vector:
    step = robot.build_step(outline)
    vel = step.solve(robot)
    dx = vel * dt
    return robot.nudge(dx)


def divide_steps[R: MovableRobot](
    robot: R,
    outlines: Iterable[Outline[R]],
    *,
    resolution: int,
) -> Generator[Vector]:
    dt = 1 / resolution
    for outline in outlines:
        step = robot.build_step(outline)
        for _ in range(resolution):
            vel = step.solve(robot)
            dx = vel * dt
            yield robot.nudge(dx)
