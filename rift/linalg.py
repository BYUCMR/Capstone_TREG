import numpy as np
from typing import TypeAlias

Vector: TypeAlias = np.ndarray[tuple[int], np.dtype[np.floating]]
Matrix: TypeAlias = np.ndarray[tuple[int, int], np.dtype[np.floating]]


def unit_vector[V: (Vector, Matrix)](v: V) -> V:
    if np.all(np.isclose(v, 0.)):
        return np.zeros_like(v)
    return v / np.linalg.norm(v)


def rotx(theta: float) -> Matrix:
    return np.array([[1., 0.           ,  0.           ],
                     [0., np.cos(theta), -np.sin(theta)],
                     [0., np.sin(theta),  np.cos(theta)]])


def roty(theta: float) -> Matrix:
    return np.array([[ np.cos(theta), 0., np.sin(theta)],
                     [ 0.           , 1., 0.           ],
                     [-np.sin(theta), 0., np.cos(theta)]])


def rotz(theta: float) -> Matrix:
    return np.array([[np.cos(theta), -np.sin(theta), 0.],
                     [np.sin(theta),  np.cos(theta), 0.],
                     [0.           ,  0.           ,  1.]])


def roll_pitch_yaw(r: float = 0, p: float = 0, y: float = 0) -> Matrix:
    return rotz(r) @ roty(p) @ rotx(y)


def rot2D(theta: float) -> Matrix:
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])

