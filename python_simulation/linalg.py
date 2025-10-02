import numpy as np
from typing import TypeAlias, TypeVar

Vector: TypeAlias = np.ndarray[tuple[int], np.dtype[np.float64]]
Matrix: TypeAlias = np.ndarray[tuple[int, int], np.dtype[np.float64]]

_ShapeT = TypeVar('_ShapeT', bound=tuple[int, ...])
_VectorOrMatrix: TypeAlias = np.ndarray[_ShapeT, np.dtype[np.float64]]
_VectorOrMatrixT = TypeVar('_VectorOrMatrixT', Vector, Matrix)


def unit_vector(v: _VectorOrMatrixT) -> _VectorOrMatrixT:
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm


def rotx(theta: float) -> Matrix:
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def roty(theta: float) -> Matrix:
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rotz(theta: float) -> Matrix:
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

def rot2D(theta: float) -> Matrix:
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])


def clean_matrix(matrix: _VectorOrMatrix[_ShapeT], *, tol: float = 1e-10) -> _VectorOrMatrix[_ShapeT]:
    matrix_cp = matrix.copy()
    matrix_cp[np.abs(matrix) < tol] = 0
    return matrix_cp
