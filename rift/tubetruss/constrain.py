import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final, Protocol, Self, SupportsIndex

import numpy as np

from rift.arraytypes import Matrix, Vector


class Constraint(Protocol):
    """A general representation of a motion constraint."""
    def get(self, x: Matrix, /) -> tuple[Matrix, Vector]: ...


@dataclass(slots=True)
class Point:
    """Representation of a linear combination of points."""
    _weights: Vector

    @classmethod
    def node(cls, i: SupportsIndex, n: SupportsIndex) -> Self:
        """Represent point `i` out of `n`."""
        weights = np.zeros(n)
        weights[i] = 1
        return cls(weights)

    @classmethod
    def diff(cls, whereto: Self, whence: Self) -> Self:
        """Represent the position of one point relative to another."""
        return cls(whereto._weights - whence._weights)

    @classmethod
    def avg(cls, *others: Self) -> Self:
        """Represent the average position of a collection of points."""
        all_weights = np.array([p._weights for p in others])
        weights = np.average(all_weights, axis=0)
        return cls(weights)

    @classmethod
    def com(cls, mass: Vector) -> Self:
        """Represent the center of mass of a collection of points."""
        return cls(mass / np.sum(mass))

    def expand(self, using: Matrix = np.eye(3)) -> Matrix:
        """
        Return a matrix that extracts this position from a flat position vector.

        The resulting matrix is effectively pre-multiplied by `using`, which
        defaults to the identity matrix.
        """
        return np.kron(self._weights, using)

    def get(self, source: Matrix) -> Vector:
        """Extract the position of this point from a position matrix."""
        return self._weights @ source

    def __sub__(self, other: Self) -> Self:
        return self.diff(self, other)


@dataclass(slots=True)
class Static:
    """A constraint that doesn't vary with position."""
    A: Matrix
    b: Vector | None = None

    @classmethod
    def combine(cls, *others: Self) -> Self:
        As: list[Matrix] = []
        bs: list[Vector] = []
        for c in others:
            As.append(c.A)
            bs.append(np.zeros(len(c.A)) if c.b is None else c.b)
        A = np.concat(As)
        b = np.concat(bs)
        return cls(A, b)

    @classmethod
    def lock(cls, point: Point) -> Self:
        """Constrain a point to be stationary."""
        return cls(point.expand())

    @classmethod
    def motion(
        cls,
        point: Point,
        directions: Matrix,
        rates: Vector,
    ) -> Self:
        """Constrain the linear motion of a point."""
        A = point.expand(directions)
        return cls(A, rates)

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

    def get(self, x: Matrix | None = None) -> tuple[Matrix, Vector]:
        b = np.zeros(len(self.A)) if self.b is None else self.b.copy()
        return self.A, b


lock: Final = Static.lock
motion: Final = Static.motion
xyz: Final = Static.xyz


@dataclass(slots=True)
class Orbit:
    """A constraint representing the rotation of some point about its origin."""
    radius: Point
    axis: Vector
    rate: float

    @classmethod
    def about_y(
        cls,
        x: Matrix,
        radius: Point,
        target: float,
        resolution: int,
    ) -> Self:
        r = radius.get(x)
        th0 = np.atan2(r[2], -r[0])
        rate = (target - th0) / resolution
        return cls(radius, np.array([0, 1, 0]), rate)

    def get(self, x: Matrix) -> tuple[Matrix, Vector]:
        r = self.radius.get(x)
        r -= (self.axis @ r) * self.axis
        v = np.cross(self.axis, r) / (r @ r)
        A = self.radius.expand(v.reshape(1, -1))
        return A, np.array([self.rate])


@dataclass(slots=True)
class ParabolicPath:
    point: Point
    origin: Vector
    rate: float
    rise: float

    @classmethod
    def make(
        cls,
        point: Point,
        *,
        init_pos: Matrix,
        delta_x: float,
        delta_y: float = 0.,
        aspect_ratio: float = 0.5,
        resolution: int,
    ) -> Self:
        origin = point.get(init_pos) + (delta_x, delta_y, 0.)
        rate = math.hypot(delta_x, delta_y) / resolution
        rise = 2. * aspect_ratio / resolution
        return cls(point, origin, rate, rise)

    def get(self, x: Matrix) -> tuple[Matrix, Vector]:
        dp = self.point.get(x) - self.origin
        r = np.linalg.norm(dp[:2])
        dp *= -self.rate / r
        dp[2] += self.rise * r
        return self.point.expand(), dp


@dataclass(slots=True)
class CompoundConstraint:
    """A constraint equivalent to a combination of other constraints."""
    constraints: Sequence[Constraint] = ()

    def get(self, x: Matrix) -> tuple[Matrix, Vector]:
        As: list[Matrix] = []
        bs: list[Vector] = []
        for c in self.constraints:
            Ai, bi = c.get(x)
            As.append(Ai)
            bs.append(bi)
        A = np.concat(As)
        b = np.concat(bs)
        return A, b


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
