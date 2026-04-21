import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Final, Protocol, Self

import numpy as np

from rift.arraytypes import Matrix, Vector
from rift.protocols import HasPos, StateFunction
from .points import Point


class Constraint[T](StateFunction[T, Matrix[np.floating]], Protocol):
    """A general representation of a motion constraint."""


@dataclass(slots=True, frozen=True)
class Compound[T](Constraint[T]):
    """A constraint equivalent to a combination of other constraints."""
    constraints: Iterable[Constraint[T]] = ()

    def at(self, state: T) -> Matrix:
       return np.concat([c.at(state) for c in self.constraints])


@dataclass(slots=True)
class Sleeper[T](Constraint[T]):
    constructor: Callable[[T], Constraint[T]]
    constraint: Constraint[T] | None = None

    def at(self, state: T) -> Matrix:
        if self.constraint is None:
            self.constraint = self.constructor(state)
        return self.constraint.at(state)


@dataclass(slots=True, frozen=True)
class Static(Constraint[object]):
    """A constraint that doesn't vary with position."""
    aug: Matrix

    @classmethod
    def combine(cls, *others: Self) -> Self:
        return cls(np.concat([c.aug for c in others]))

    @classmethod
    def make_hom(cls, A: Matrix) -> Self:
        m, n = A.shape
        aug = np.zeros((m, n+1))
        aug[:, :-1] = A
        return cls(aug)

    @classmethod
    def lock(cls, point: Point) -> Self:
        """Constrain a point to be stationary."""
        return cls.make_hom(np.kron(point, np.eye(3)))

    @classmethod
    def motion(
        cls,
        point: Point,
        directions: Matrix,
        rates: Vector,
    ) -> Self:
        """Constrain the linear motion of a point."""
        A = np.kron(point, directions)
        aug = np.column_stack((A, rates))
        return cls(aug)

    @classmethod
    def xyz(
        cls,
        point: Point,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
    ) -> Self:
        """Set the motion of a point along the x, y, and z axes."""
        i = [e is not None for e in (x, y, z)]
        directions = np.eye(3)[i]
        rates = np.array([e for e in (x, y, z) if e is not None])
        return cls.motion(point, directions, rates)

    def at(self, state: object = None) -> Matrix:
        return self.aug.copy()


lock: Final = Static.lock
motion: Final = Static.motion
xyz: Final = Static.xyz


@dataclass(slots=True, frozen=True)
class PlanarBarrier(Constraint[HasPos]):
    """As an inequality constraint, this keeps nodes on one side of a plane."""
    points: Matrix
    normal: Vector

    def at(self, state: HasPos) -> Matrix:
        A = np.kron(self.points, -self.normal)
        b = self.points @ state.pos @ self.normal
        return np.column_stack((A, b))


@dataclass(slots=True, frozen=True)
class Radial(Constraint[HasPos]):
    radius: Point
    rate: float

    @classmethod
    def get_to(
        cls,
        radius: Point,
        length: float,
        *,
        init_pos: Matrix | None = None,
    ) -> Self | Sleeper[HasPos]:
        if init_pos is None:
            return Sleeper(lambda state: cls.get_to(radius, length, init_pos=state.pos))
        current_length = np.linalg.norm(radius @ init_pos)
        rate = length - current_length
        return cls(radius, float(rate))

    def at(self, state: HasPos) -> Matrix:
        r = self.radius @ state.pos
        r /= np.linalg.norm(r)
        A = np.kron(self.radius, r.reshape(1, -1))
        return np.column_stack((A, self.rate))


@dataclass(slots=True, frozen=True)
class Orbit(Constraint[HasPos]):
    """A constraint representing the rotation of some point about its origin."""
    radius: Point
    axis: Vector
    rate: float

    @classmethod
    def align(
        cls,
        radius: Point,
        end: Vector,
        *,
        axis: Vector | None = None,
        init_pos: Matrix | None = None,
    ) -> Self | Sleeper[HasPos]:
        if init_pos is None:
            return Sleeper(lambda state: cls.align(radius, end, axis=axis, init_pos=state.pos))
        start = radius @ init_pos
        if axis is not None:
            start -= (axis @ start) * axis
            end = end - (axis @ end) * axis
        axis = -np.cross(start, end)
        axis_norm = np.linalg.norm(axis)
        angle = math.asin(axis_norm / (np.linalg.norm(start) * np.linalg.norm(end)))
        axis /= axis_norm
        return cls(radius, axis, -angle)

    def at(self, state: HasPos) -> Matrix:
        r = self.radius @ state.pos
        r -= (self.axis @ r) * self.axis
        v = np.cross(self.axis, r) / (r @ r)
        A = np.kron(self.radius, v.reshape(1, -1))
        return np.column_stack((A, self.rate))


@dataclass(slots=True, frozen=True)
class ParabolicPath(Constraint[HasPos]):
    point: Point
    origin: Vector
    rate: float
    rise: float

    @classmethod
    def make(
        cls,
        point: Point,
        *,
        delta_x: float,
        delta_y: float = 0.,
        aspect_ratio: float = 0.5,
        init_pos: Matrix | None = None,
    ) -> Self | Sleeper[HasPos]:
        if init_pos is None:
            return Sleeper(lambda state: cls.make(
                point=point,
                delta_x=delta_x,
                delta_y=delta_y,
                aspect_ratio=aspect_ratio,
                init_pos=state.pos,
            ))
        origin = point @ init_pos + (delta_x, delta_y, 0.)
        rate = math.hypot(delta_x, delta_y)
        rise = 2. * aspect_ratio
        return cls(point, origin, rate, rise)

    def at(self, state: HasPos) -> Matrix:
        dp = self.point @ state.pos - self.origin
        r = np.linalg.norm(dp[:2])
        dp *= -self.rate / r
        dp[2] += self.rise * r
        A = np.kron(self.point, np.eye(3))
        return np.column_stack((A, dp))


def combine[T](*constraints: Constraint[T]) -> Constraint[T]:
    if not constraints:
        raise ValueError("Cannot combine zero constraints")
    all_constraints = list(constraints)
    static: list[Static] = []
    dynamic: list[Constraint[T]] = []
    for constraint in all_constraints:
        match constraint:
            case Static():
                static.append(constraint)
            case Compound(subconstraints):
                all_constraints.extend(subconstraints)
            case Sleeper(constraint=subconstraint) if subconstraint is not None:
                all_constraints.append(subconstraint)
            case _:
                dynamic.append(constraint)
    if static:
        dynamic.append(Static.combine(*static))
    if len(dynamic) == 1:
        return dynamic.pop()
    return Compound(dynamic)
