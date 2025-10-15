import numpy as np
from collections.abc import Generator, Sequence
from dataclasses import dataclass
from typing import Final, TypeAlias

from linalg import Matrix

Triangles: TypeAlias = Sequence[tuple[int, int, int]]


@dataclass(kw_only=True)
class TrussConfig:
    supports: Sequence[int]
    move_node: int
    triangles: Triangles
    initial_pos: Matrix


def edges(triangles: Triangles) -> Generator[tuple[int, int]]:
    for (n1, n2, n3) in triangles:
        yield (n1, n2)
        yield (n2, n3)
        yield (n3 ,n1)


CONFIG_3D_1: Final = TrussConfig(
    supports=[1, 5, 3],
    move_node=0,
    triangles=[(0,2,3), (0,4,5), (1,2,4), (1,3,5)],
    initial_pos=np.array([
        [7.07106781,  4.0824829 , 5.77350269],
        [0.        ,  0.        , 0.        ],
        [0.        ,  4.0824829 , 5.77350269],
        [3.53553391,  6.12372436, 0.        ],
        [3.53553391, -2.04124145, 5.77350269],
        [7.07106781,  0.        , 0.        ],
    ]),
)

CONFIG_3D_2: Final = TrussConfig(
    supports=[1, 5, 3],
    move_node=0,
    triangles=[(0,2,4), (0,3,5), (1,2,3), (1,4,5)],
    initial_pos=np.array([
        [7.07106781,  4.0824829 , 5.77350269],
        [0.        ,  0.        , 0.        ],
        [0.        ,  4.0824829 , 5.77350269],
        [3.53553391,  6.12372436, 0.        ],
        [3.53553391, -2.04124145, 5.77350269],
        [7.07106781,  0.        , 0.        ],
    ]),
)


CONFIG_2D_1: Final = TrussConfig(
    supports=[0, 1],
    move_node=2,
    triangles=[(0,1,2)],
    initial_pos=np.array([
        [0.        , 0.        ],
        [7.07106781, 0.        ],
        [3.53553391, 6.12372436],
    ]),
)

CONFIG_2D_2: Final = TrussConfig(
    supports=[0, 3],
    move_node=5,
    triangles=[(0,1,2), (1,3,4), (2,4,5)],
    initial_pos=np.array([
        [ 0.        ,  0.        ],
        [ 7.07106781,  0.        ],
        [ 3.53553391,  6.12372436],
        [14.14213562,  0.        ],
        [10.60660172,  6.12372436],
        [ 7.07106781, 12.24744871],
    ]),
)
