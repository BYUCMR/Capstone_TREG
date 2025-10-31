import numpy as np
from collections.abc import Generator, Collection
from dataclasses import dataclass, field
from typing import Final, SupportsIndex, TypeAlias

from linalg import Matrix

Index: TypeAlias = 'SupportsIndex | slice[SupportsIndex]'
Triangles: TypeAlias = Collection[tuple[int, int, int]]
Edges: TypeAlias = Collection[tuple[int, int]]


@dataclass(slots=True, kw_only=True, frozen=True)
class TrussConfig:
    locks: Collection[tuple[Index, Index]] = field(default_factory=list)
    move_node: int
    payload: Edges
    triangles: Triangles
    initial_pos: Matrix


def edges(triangles: Triangles) -> Generator[tuple[int, int]]:
    for (n1, n2, n3) in triangles:
        yield (n1, n2)
        yield (n2, n3)
        yield (n3, n1)


def get_support_indices(config: TrussConfig) -> np.ndarray[tuple[int], np.dtype[np.intp]]:
    s = np.zeros_like(config.initial_pos)
    for lock in config.locks:
        s[lock] = 1
    indices, = s.ravel(order='F').nonzero()
    return indices


CONFIG_3D_ROVER1: Final = TrussConfig(
    locks=[(0, slice(0,3)), (1, slice(0,3)), (6, slice(0,3))],
    move_node=7,
    triangles=np.array([[0, 1, 2], [0, 3, 5], [1, 4, 5], [6, 7, 8], [6, 9, 11], [7, 10, 11]]),
    payload=[(2, 8), (3, 9), (4, 10), (2, 9), (3, 10), (4, 8),(2,3),(3,4),(2,4),(8,10),(8,9),(9,10)],
    initial_pos=np.array([  # Side 1
        [12.879, 4.003, -10.963],  # Ground (0)
        [12.879, -13.997, -10.963],  # Ground (1)
        [7.185, -4.997, -5.433],  # Payload (2)
        [7.185, -1.533, .567],  # Payload (3)
        [7.185, -8.461, .567],  # Payload (4)
        [20.229, -4.997, -3.154],  # Float (5)
        # side 2
        [-10.709, 4.003, -10.963],  # Ground (6)
        [-10.709, -13.997, -10.963],  # Ground (7)
        [-5.015, -4.997, -5.433],  # Payload (8)
        [-5.015, -1.533, .567],  # Payload (9)
        [-5.015, -8.461, .567],  # Payload (10)
        [-18.059, -4.997, -3.154]  # Float (11)
    ]),
)

CONFIG_3D_1: Final = TrussConfig(
    locks=[(1, slice(0,3)), (5, slice(1,3)), (3, slice(2,3))],
    move_node=0,
    payload=[],
    triangles=[(0, 2, 3), (0, 4, 5), (1, 2, 4), (1, 3, 5)],
    initial_pos=np.array([
        [7.07106781, 4.0824829, 5.77350269],
        [0., 0., 0.],
        [0., 4.0824829, 5.77350269],
        [3.53553391, 6.12372436, 0.],
        [3.53553391, -2.04124145, 5.77350269],
        [7.07106781, 0., 0.],
    ]),
)

CONFIG_3D_2: Final = TrussConfig(
    locks=[(1, slice(0,3)), (5, slice(1,3)), (3, slice(2,3))],
    move_node=0,
    payload=[],
    triangles=[(0, 2, 4), (0, 3, 5), (1, 2, 3), (1, 4, 5)],
    initial_pos=np.array([
        [7.07106781, 4.0824829, 5.77350269],
        [0., 0., 0.],
        [0., 4.0824829, 5.77350269],
        [3.53553391, 6.12372436, 0.],
        [3.53553391, -2.04124145, 5.77350269],
        [7.07106781, 0., 0.],
    ]),
)

CONFIG_2D_1: Final = TrussConfig(
    locks=[(0, slice(0,2)), (1, slice(1,2))],
    move_node=2,
    payload=[],
    triangles=[(0, 1, 2)],
    initial_pos=np.array([
        [0., 0.],
        [7.07106781, 0.],
        [3.53553391, 6.12372436],
    ]),
)

CONFIG_2D_2: Final = TrussConfig(
    locks=[(0, slice(0,2)), (3, slice(1,2))],
    move_node=5,
    payload=[],
    triangles=[(0, 1, 2), (1, 3, 4), (2, 4, 5)],
    initial_pos=np.array([
        [0., 0.],
        [7.07106781, 0.],
        [3.53553391, 6.12372436],
        [14.14213562, 0.],
        [10.60660172, 6.12372436],
        [7.07106781, 12.24744871],
    ]),
)
