from dataclasses import dataclass
from typing import Self

import numpy as np

from rift.arraytypes import Matrix


@dataclass(frozen=True)
class LengthControl:
    unreachable: Matrix[np.integer]
    forward: Matrix[np.integer]
    inverse: Matrix

    @property
    def n_inputs(self) -> int:
        return len(self.inverse)

    @property
    def n_outputs(self) -> int:
        return len(self.forward)

    @classmethod
    def from_unreachable(cls, unreachable: Matrix[np.integer]) -> Self:
        forward = cokernel(unreachable.T).T
        inverse = np.linalg.pinv(forward)
        return cls(unreachable, forward, inverse)

    @classmethod
    def from_forward(cls, forward: Matrix[np.integer]) -> Self:
        inverse = np.linalg.pinv(forward)
        unreachable = cokernel(forward)
        return cls(unreachable, forward, inverse)


def cokernel[T: np.integer](mat: Matrix[T]) -> Matrix[T]:
    """
    Return a basis for the co-kernel of a matrix.

    The co-kernel of a matrix is the null space, or kernel, of its transpose.
    Each column of a basis for the co-kernel is perpendicular to every row of
    the matrix.
    """
    m, n = mat.shape
    eye = np.identity(m, dtype=mat.dtype)
    aug = np.concat((mat, eye), axis=1)
    row = 0
    col = 0
    while True:
        rows, cols = np.nonzero(aug[row:, col:n])
        if len(rows) == 0:
            break
        leftmost = np.argmin(cols)
        col += cols[leftmost]
        other_row = row + rows[leftmost]
        if other_row != row:
            aug[[row, other_row]] = aug[[other_row, row]]
        pivot = aug[row]
        row += 1
        beneath = aug[row:]
        beneath[:] = pivot[col]*beneath - np.outer(beneath[:,col], pivot)
    return aug[row:, n:]
