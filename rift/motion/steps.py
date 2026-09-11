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
    """
    Return an eigenvalue and eigenvector corresponding to
    the proximity of the given system to a singularity.
    """
    evals, evecs = np.linalg.eigh(A.T @ A)
    m, n = A.shape
    if b is not None:
        aug = np.concat((A, b.reshape(-1, 1)), axis=1)
        # Note that this takes about as much time as getting the eigenvalues;
        # it's better to develop minimal constraints by hand.
        m = np.linalg.matrix_rank(aug)
    i = max(0, n - m)
    return evals[i], evecs[:, i]


class AbstractOutline[T](Protocol):
    @property
    def eq(self, /) -> cstr.Constraint[T] | None: ...
    @property
    def le(self, /) -> cstr.Constraint[T] | None: ...
    @property
    def allow_redundancy(self, /) -> bool: ...


@dataclass(slots=True, frozen=True)
class Outline[T]:
    """A high-level description of a step for a robot to take."""
    eq: cstr.Constraint[T] | None = None
    le: cstr.Constraint[T] | None = None
    allow_redundancy: bool = field(default=False, kw_only=True)


def combine_outlines[T](lhs: AbstractOutline[T], rhs: AbstractOutline[T]) -> Outline[T]:
    if lhs.eq is None:
        eq = rhs.eq
    elif rhs.eq is None:
        eq = lhs.eq
    else:
        eq = cstr.combine(lhs.eq, rhs.eq)
    if lhs.le is None:
        le = rhs.le
    elif rhs.le is None:
        le = lhs.le
    else:
        le = cstr.combine(lhs.le, rhs.le)
    allow_redundancy = lhs.allow_redundancy or rhs.allow_redundancy
    return Outline(eq, le, allow_redundancy=allow_redundancy)


class Step[T](Protocol):
    """A basic interface for a step for a robot to take."""
    def solve(self, state: T, /) -> Vector: ...


class CanStep(Protocol):
    """A basic interface for a robot that can take steps."""
    def build_step(self, outline: AbstractOutline[Self], /) -> Step[Self]: ...
    def nudge(self, change: Vector, /) -> Vector: ...


@dataclass(slots=True)
class QPStep[T](Step[T]):
    """A step that selects optimal motion by solving a quadratic program."""
    quad_cost: StateFunction[T, Matrix]
    lin_cost: StateFunction[T, Vector] | None
    outline: AbstractOutline[T]

    def solve(self, state: T) -> Vector:
        if self.outline.eq is not None:
            Ab = self.outline.eq.at(state)
            e, v = singularity_eig(Ab[:, :-1], Ab[:, -1] if self.outline.allow_redundancy else None)
            if abs(e) <= 1e-3:
                raise SingularityError("Robot state is singular")
        else:
            Ab = None
        Gh = (
            None if self.outline.le is None
            else self.outline.le.at(state)
        )
        R = self.quad_cost.at(state)
        f = None if self.lin_cost is None else self.lin_cost.at(state)
        if Gh is not None or self.outline.allow_redundancy:
            vel = optimize.solve_qp(R=R, f=f, Ab=Ab, Gh=Gh, solver='piqp')
        elif Ab is not None and Ab.shape[1] == Ab.shape[0]+1:
            try:
                vel = np.linalg.solve(Ab[:, :-1], Ab[:, -1])
            except np.linalg.LinAlgError:
                vel = None
        else:
            vel = optimize.solve_kkt(R=R, f=f, Ab=Ab)
        if vel is None:
            raise SolverError("Could not find valid node velocities")
        return vel


def divide_steps[R: CanStep](
    robot: R,
    outlines: Iterable[AbstractOutline[R]],
    *,
    resolution: int,
) -> Generator[Vector]:
    """Divide a sequence of steps into one of finer resolution."""
    dt = 1 / resolution
    for outline in outlines:
        step = robot.build_step(outline)
        for _ in range(resolution):
            vel = step.solve(robot)
            dx = vel * dt
            yield robot.nudge(dx)
