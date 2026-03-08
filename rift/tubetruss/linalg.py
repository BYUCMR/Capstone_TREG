import math
from collections.abc import Iterable
from itertools import chain, pairwise
from typing import cast

import numpy as np

from rift.arraytypes import Matrix, SingleIndex, Vector


def incidence_from_trails(
    *trails: Iterable[SingleIndex],
    empty_cols: int = 0,
) -> Matrix[np.int8]:
    n_cols = 1 + max(chain.from_iterable(trails)) + empty_cols
    rows: list[Vector[np.int8]] = []
    for trail in trails:
        for i, j in pairwise(trail):
            row = np.zeros(n_cols, dtype=np.int8)
            row[i] = 1
            row[j] = -1
            rows.append(row)
    return np.array(rows)


def reduce[T: np.integer](mat: Matrix[T], *, in_place: bool = False) -> Matrix[T]:
    """Convert an integer matrix to a row echelon form."""
    if not in_place:
        mat = mat.copy()
    block = mat
    while len(cols := np.nonzero(block)[1]):
        block = block[:, np.min(cols):]
        if row := np.argmax(np.abs(block[:, 0])):
            # This is slower than just swapping two rows, but it
            # leads to better-structured results for our use case.
            copied = block[0:row].copy()
            block[0] = block[row]
            block[1:row+1] = copied
        pivot = block[0]
        if pivot[0] < 0:
            pivot *= -1
        pivot //= math.gcd(*pivot)
        block = block[1:]
        block[:] = pivot[0]*block - np.outer(block[:,0], pivot)
    return mat


def cokernel[T: np.number](mat: Matrix[T]) -> Matrix[T]:
    """
    Return the co-kernel of a matrix in the form of a basis of row vectors.

    The co-kernel of a matrix is the null space, or kernel, of its transpose.
    It is the orthogonal complement to the column space of the matrix.

    Thus, `cokernel(mat) @ mat` will be a matrix of 0s.
    """
    m, n = mat.shape
    if np.issubdtype(mat.dtype, np.integer):
        eye = np.eye(m, dtype=mat.dtype)
        aug = np.concat((mat, eye), axis=1)
        # The type-checker has trouble inferring that we know
        # this is an integer matrix, so we use typing.cast here.
        reduce(cast(Matrix[np.integer], aug), in_place=True)
        end = 1 + np.max(np.nonzero(aug[:, :n])[0])
        return aug[end:, n:]
    else:
        U, S, Vh = np.linalg.svd(mat.T, full_matrices=True)
        tol = np.max(S) * max(m, n) * np.finfo(S.dtype).eps
        rank = np.count_nonzero(S > tol)
        return Vh[rank:, :].conj()
