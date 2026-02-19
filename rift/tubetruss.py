import math
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from functools import cached_property
from itertools import pairwise
from typing import Collection, Self, cast

import numpy as np
from scipy.linalg import block_diag as _scipy_block_diag

from .arraytypes import Matrix, MatrixStack, Vector


def block_diag[T: np.generic](mats: Iterable[Matrix[T] | Vector[T]]) -> Matrix[T]:
    return cast(Matrix[T], _scipy_block_diag(*mats))


type Node = int
type Link = tuple[Node, Node]


@dataclass(slots=True, frozen=True)
class Tube:
    start: Node
    rollers: tuple[Node, ...]
    stop: Node

    @classmethod
    def make_bar(cls, start: Node, stop: Node) -> Self:
        return cls(start, (), stop)

    @classmethod
    def make_tri(cls, start: Node, roll_1: Node, roll_2: Node) -> Self:
        return cls(start, (roll_1, roll_2), start)

    @property
    def nodes(self) -> tuple[Node, ...]:
        return self.start, *self.rollers, self.stop

    @property
    def links(self) -> Iterator[Link]:
        if self.rollers:
            yield self.start, self.rollers[0]
            yield from pairwise(self.rollers)
            yield self.rollers[-1], self.stop
        else:
            yield self.start, self.stop


@dataclass(frozen=True)
class TubeTruss:
    tubes: Collection[Tube] = ()

    @classmethod
    def make_bars(cls, tubes: Iterable[tuple[int, int]]) -> Self:
        return cls([Tube.make_bar(a, b) for a, b in tubes])

    @classmethod
    def make_tris(cls, tubes: Iterable[tuple[int, int, int]]) -> Self:
        return cls([Tube.make_tri(a, b, c) for a, b, c in tubes])

    @property
    def nodes(self) -> Iterator[Node]:
        for tube in self.tubes:
            yield from tube.nodes

    @property
    def links(self) -> Iterator[Link]:
        for tube in self.tubes:
            yield from tube.links

    @cached_property
    def incidence(self) -> Matrix[np.intp]:
        """See `get_incidence`."""
        return get_incidence(self)

    @cached_property
    def length_summer(self) -> Matrix[np.bool]:
        """See `get_length_summer`."""
        return get_length_summer(self)

    @cached_property
    def pos_to_rigidity(self) -> MatrixStack:
        """See `get_pos_to_rigidity`."""
        return get_pos_to_rigidity(self)

    def rigidity_at(self, pos: Matrix | Vector) -> Matrix:
        """See `get_rigidity`."""
        return self.pos_to_rigidity @ pos.ravel()

    def norm_rigidity_at(self, pos: Matrix | Vector) -> Matrix:
        """See `normalize_rigidity`."""
        return normalize_rigidity(self.rigidity_at(pos))

    def __iter__(self) -> Iterator[Tube]:
        return iter(self.tubes)

    def __len__(self) -> int:
        return len(self.tubes)

    def __add__(self, other: Self) -> Self:
        return type(self)([*self, *other])

    __radd__ = __add__


def get_incidence(structure: TubeTruss) -> Matrix[np.intp]:
    """Return the transpose of the incidence matrix for a truss."""
    n_nodes = 1 + max(structure.nodes)
    rows: list[Vector[np.intp]] = []
    for n1, n2 in structure.links:
        row = np.zeros(n_nodes, dtype=np.intp)
        row[n1] =  1
        row[n2] = -1
        rows.append(row)
    return np.array(rows, dtype=int)


def get_rigidity(structure: TubeTruss, pos: Matrix) -> Matrix:
    """Return the rigidity matrix of a tube structure in a given state.

    This matrix converts a column of node velocities into a column comprising
    the length of each link times its rate of change.
    """
    rows: list[Vector] = []
    for i in get_incidence(structure):
        row = np.linalg.outer(i, i) @ pos
        rows.append(row.ravel())
    return np.array(rows)


def get_pos_to_rigidity(structure: TubeTruss, *, dim: int = 3) -> MatrixStack:
    """Return a 3-D matrix that can be used to calculate a rigidity matrix.

    This matrix converts a vector of node positions into a rigidity matrix.
    The rigidity matrix converts a column of node velocities into a column
    comprising the length of each link times its rate of change.
    """
    B = get_incidence(structure)
    M = np.array([np.linalg.outer(row, row) for row in B])
    # We could skip the Kronecker product here and replace
    # `M @ pos.ravel()` with `(M @ pos).reshape(-1, pos.size)`
    # later on, but this keeps the interface simpler.
    return np.kron(M, np.eye(dim))


def normalize_rigidity(rigidity: Matrix) -> Matrix:
    """Return a normalized copy of a rigidity matrix.

    This matrix converts a column of node velocities into a column comprising
    the rate of change of the length of each link.
    """
    norms = np.linalg.vector_norm(rigidity, axis=1, keepdims=True)
    return math.sqrt(2.) * rigidity / norms


def get_length_summer(structure: TubeTruss) -> Matrix[np.bool]:
    """Return a matrix that sums link lengths into total tube lengths."""
    return block_diag(
        np.ones(len(tube.nodes)-1, dtype=np.bool) for tube in structure
    )
