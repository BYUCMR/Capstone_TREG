from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from functools import cached_property
from itertools import pairwise
from typing import Collection, Self, cast

import numpy as np
from scipy.linalg import block_diag as _scipy_block_diag

from .linalg import Matrix, Vector, unit_vector
from .state import RobotState


def block_diag(mats: Iterable[np.typing.ArrayLike]) -> np.ndarray:
    return cast(Matrix, _scipy_block_diag(*mats))


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
    def incidence(self) -> Matrix:
        """See `get_incidence`."""
        return get_incidence(self)

    @cached_property
    def incidence_inv(self) -> Matrix:
        """See `get_incidence_inv`."""
        return get_incidence_inv(self)

    @cached_property
    def length_constraint(self) -> Matrix:
        """See `get_length_constraint`."""
        return get_length_constraint(self)

    def rigidity_at(self, state: RobotState) -> Matrix:
        """See `get_rigidity`."""
        return get_rigidity(self, state)

    def __iter__(self) -> Iterator[Tube]:
        return iter(self.tubes)

    def __len__(self) -> int:
        return len(self.tubes)

    def __add__(self, other: Self) -> Self:
        return type(self)([*self, *other])

    __radd__ = __add__


def get_rigidity(structure: TubeTruss, state: RobotState) -> Matrix:
    """Return the rigidity matrix of a tube structure in a given state.

    This matrix converts a column of node velocities into a column of the rate of
    change of the length of each link.
    """
    dim = state.pos.shape[1]
    j = np.arange(dim)
    rows: list[Vector] = []
    for n1, n2 in structure.links:
        row = np.zeros(state.pos.size)
        u = unit_vector(state.pos[n1] - state.pos[n2])
        row[dim*n1 + j] = u
        row[dim*n2 + j] = -u
        rows.append(row)
    return np.array(rows)


def get_incidence(structure: TubeTruss) -> Matrix:
    """Return a matrix describing how each roller affects the length of each link.

    This matrix converts a column of roller roll values into a column of link lengths.
    It is the transpose of the incidence matrix of links to rollers, but some links are
    only attached to a single roller.
    """
    mats: list[Matrix] = []
    for tube in structure:
        n_roll = len(tube.rollers)
        mat = np.zeros((n_roll+1, n_roll))
        i, j = np.diag_indices(n_roll)
        mat[i  , j] = -1
        mat[i+1, j] =  1
        mats.append(mat)
    return block_diag(mats)


def get_incidence_inv(structure: TubeTruss) -> Matrix:
    """Return a matrix describing how each roller affects the length of each link.

    This matrix converts a column of link lengths into a column of roller roll values.
    It is the pseudo-inverse of the matrix returned by `get_incidence`.
    """
    mats: list[Matrix] = []
    for tube in structure:
        n_roll = len(tube.rollers)
        n_link = n_roll + 1
        mat = np.full((n_roll, n_link), 1 / n_link)
        mat *= np.arange(1, n_link).reshape(n_roll, 1)
        mat -= np.tri(n_link)[:-1]
        mats.append(mat)
    return block_diag(mats)


def get_length_constraint(structure: TubeTruss) -> Matrix:
    """Return a matrix that converts link lengths to total tube lengths."""
    return block_diag([1. for _ in tube.links] for tube in structure)
