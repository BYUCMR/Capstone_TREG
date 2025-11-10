import numpy as np
from collections.abc import Collection
from dataclasses import dataclass, field
from typing import Final, SupportsIndex, TypeAlias

from .linalg import Matrix
from .tubetruss import TubeTruss

Index: TypeAlias = 'SupportsIndex | slice[SupportsIndex]'
Lock: TypeAlias = tuple[Index, Index]
Link: TypeAlias = tuple[int, int]
Triangle: TypeAlias = tuple[int, int, int]


bars = TubeTruss.make_bars
tris = TubeTruss.make_tris


@dataclass(slots=True, kw_only=True, frozen=True)
class TrussConfig:
    locks: Collection[Lock] = ()
    move_node: int
    payload: TubeTruss = field(default_factory=TubeTruss)
    triangles: TubeTruss = field(default_factory=TubeTruss)
    initial_pos: Matrix


CONFIG_3D_ROVER1: Final = TrussConfig(
    locks=[(0, slice(0,3)), (1, slice(0,3)), (6, slice(0,3))],
    move_node=7,
    triangles=tris([(0, 1, 2), (0, 3, 5), (1, 4, 5), (6, 7, 8), (6, 9, 11), (7, 10, 11)]),
    payload=bars([(2,8), (3,9), (4,10), (2,9), (3,10), (4,8), (2,3), (3,4), (2,4), (8,10), (8,9), (9,10)]),
    initial_pos=np.array([
        # Side 1
        [  0.   ,  11.794,   0.   ],  # Ground  ( 0)
        [-18.   ,  11.794,   0.   ],  # Ground  ( 1)
        [ -9.   ,   6.1  ,   5.53 ],  # Payload ( 2)
        [ -5.536,   6.1  ,  11.53 ],  # Payload ( 3)
        [-12.464,   6.1  ,  11.53 ],  # Payload ( 4)
        [ -9.   ,  19.144,   7.809],  # Float   ( 5)
        # Side 2
        [  0.   , -11.794,   0.   ],  # Ground  ( 6)
        [-18.   , -11.794,   0.   ],  # Ground  ( 7)
        [ -9.   ,  -6.1  ,   5.53 ],  # Payload ( 8)
        [ -5.536,  -6.1  ,  11.53 ],  # Payload ( 9)
        [-12.464,  -6.1  ,  11.53 ],  # Payload (10)
        [ -9.   , -19.144,   7.809],  # Float   (11)
    ]),
)

CONFIG_3D_1: Final = TrussConfig(
    locks=[(1, slice(0,3)), (5, slice(1,3)), (3, slice(2,3))],
    move_node=0,
    triangles=tris([(0, 2, 3), (0, 4, 5), (1, 2, 4), (1, 3, 5)]),
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
    triangles=tris([(0, 2, 4), (0, 3, 5), (1, 2, 3), (1, 4, 5)]),
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
    triangles=tris([(0, 1, 2)]),
    initial_pos=np.array([
        [0., 0.],
        [7.07106781, 0.],
        [3.53553391, 6.12372436],
    ]),
)

CONFIG_2D_2: Final = TrussConfig(
    locks=[(0, slice(0,2)), (3, slice(1,2))],
    move_node=5,
    triangles=tris([(0, 1, 2), (1, 3, 4), (2, 4, 5)]),
    initial_pos=np.array([
        [0., 0.],
        [7.07106781, 0.],
        [3.53553391, 6.12372436],
        [14.14213562, 0.],
        [10.60660172, 6.12372436],
        [7.07106781, 12.24744871],
    ]),
)
