from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Protocol, Self

import numpy as np

from .arraytypes import Matrix, Vector


type CanCall[T] = Callable[[float], T] | T


class Constraint(Protocol):
    """A general representation of a motion constraint."""
    def get(self, x: Matrix, t: float, /) -> tuple[Matrix, Vector]: ...


@dataclass(slots=True)
class Point:
    """Representation of a linear combination of points."""
    _weights: Vector

    @classmethod
    def node(cls, i: int, n: int) -> Self:
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

    def expand(self, using: Matrix | Vector = np.eye(3)) -> Matrix:
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
class CustomConstraint:
    """A custom constraint that doesn't vary with position or time."""
    A: Matrix
    b: Vector | None = None

    def get(self, x: Matrix, t: float) -> tuple[Matrix, Vector]:
        b = np.zeros(len(self.A)) if self.b is None else self.b
        return self.A, b


@dataclass(slots=True)
class Motion:
    """A constraint representing the linear motion of a point."""
    point: Point
    direction: Vector | Matrix = field(default_factory=lambda: np.eye(3))
    v: CanCall[float | Vector] = field(default_factory=lambda: np.zeros(3))

    @classmethod
    def make(
        cls,
        point: Point,
        x: CanCall[float] | None = None,
        y: CanCall[float] | None = None,
        z: CanCall[float] | None = None,
    ) -> Self:
        """Set the motion of a point along the x, y, and z axes."""
        i = [e is not None for e in (x, y, z)]
        if not np.any(i):
            return cls(point, np.zeros(3), np.zeros(0))
        def b(t: float) -> Vector:
            rows: list[float] = []
            for v in (x, y, z):
                if v is None:
                    continue
                elif callable(v):
                    row = v(t)
                else:
                    row = v
                rows.append(row)
            return np.array(rows)
        return cls(point, np.eye(3)[i], b)

    @classmethod
    def lock(cls, point: Point) -> Self:
        """Constrain a point to be stationary."""
        return cls(point)

    def get(self, x: Matrix, t: float) -> tuple[Matrix, Vector]:
        A = self.point.expand(self.direction)
        b = np.atleast_1d(self.v(t) if callable(self.v) else self.v)
        return A, b


@dataclass(slots=True)
class Orbit:
    """A constraint representing the rotation of some point about its origin."""
    radius: Point
    axis: Vector
    rate: CanCall[float]

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

    def get(self, x: Matrix, t: float) -> tuple[Matrix, Vector]:
        r = self.radius.get(x)
        r -= (self.axis @ r) * self.axis
        M = np.cross(self.axis, r) / (r @ r)
        A = self.radius.expand(M)
        b = self.rate(t) if callable(self.rate) else self.rate
        return A, np.array([b])


@dataclass(slots=True)
class CompoundConstraint:
    """A constraint equivalent to a combination of other constraints."""
    constraints: Sequence[Constraint] = ()

    def get(self, x: Matrix, t: float) -> tuple[Matrix, Vector]:
        As: list[Matrix] = []
        bs: list[Vector] = []
        for c in self.constraints:
            Ai, bi = c.get(x, t)
            As.append(Ai)
            bs.append(bi)
        A = np.vstack(As)
        b = np.concat(bs)
        return A, b


def singularity_eig(A: Matrix, b: Vector | None = None) -> tuple[float, Vector]:
    evals, evecs = np.linalg.eigh(A.T @ A)
    m, n = A.shape
    if b is not None:
        aug = np.hstack((A, b.reshape(-1, 1)))
        # Note that this takes about as much time as getting the eigenvalues;
        # it's better to develop minimal constraints by hand.
        m = np.linalg.matrix_rank(aug)
    i = max(0, n - m)
    return evals[i], evecs[:, i]
