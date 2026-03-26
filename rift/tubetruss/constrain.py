import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final, Protocol, Self

import numpy as np

from rift.arraytypes import Matrix, Vector


X: Final = np.array((1., 0., 0.))
Y: Final = np.array((0., 1., 0.))
Z: Final = np.array((0., 0., 1.))
X.setflags(write=False)
Y.setflags(write=False)
Z.setflags(write=False)


class Point(Protocol):
    """
    A general representation of a point on a truss.

    `__array__` should return a vector of weights describing how much each
    node of the truss contributes to this point's position.
    """
    def __array__(self, /) -> Vector: ...


class Constraint(Protocol):
    """A general representation of a motion constraint."""
    def get(self, x: Matrix, /) -> tuple[Matrix, Vector]: ...


def centroid(*points: Point) -> Vector:
    """Return the mean of the input points."""
    return np.average(np.array(points), axis=0)


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
        return cls(np.kron(point, np.eye(3)))

    @classmethod
    def motion(
        cls,
        point: Point,
        directions: Matrix,
        rates: Vector,
    ) -> Self:
        """Constrain the linear motion of a point."""
        A = np.kron(point, directions)
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
        r = radius @ x
        th0 = np.atan2(r[2], -r[0])
        rate = (target - th0) / resolution
        return cls(radius, Y, rate)

    def get(self, x: Matrix) -> tuple[Matrix, Vector]:
        r = self.radius @ x
        r -= (self.axis @ r) * self.axis
        v = np.cross(self.axis, r) / (r @ r)
        A = np.kron(self.radius, v.reshape(1, -1))
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
        origin = point @ init_pos + (delta_x, delta_y, 0.)
        rate = math.hypot(delta_x, delta_y) / resolution
        rise = 2. * aspect_ratio / resolution
        return cls(point, origin, rate, rise)

    def get(self, x: Matrix) -> tuple[Matrix, Vector]:
        dp = self.point @ x - self.origin
        r = np.linalg.norm(dp[:2])
        dp *= -self.rate / r
        dp[2] += self.rise * r
        A = np.kron(self.point, np.eye(3))
        return A, dp


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
