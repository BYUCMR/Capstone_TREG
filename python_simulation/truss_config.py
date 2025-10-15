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
        [ 1.33408391780635,  -0.115416570167785,  4.81735395373655 ],
        [-1.33408391780635,   0.115416570167783, -4.81735395373654 ],
        [ 4.81591881210246,   0.202844182067796, -1.32882663769163 ],
        [ 0.164760721921360, -4.99455035545062,  -0.165289598523321],
        [-0.164760721921362,  4.99455035545062,   0.165289598523319],
        [-4.81591881210246,  -0.202844182067798,  1.32882663769163 ],
    ]),
)

CONFIG_3D_2: Final = TrussConfig(
    supports=[1, 5, 3],
    move_node=0,
    triangles=[(0,2,4), (0,3,5), (1,2,3), (1,4,5)],
    initial_pos=np.array([
        [ 1.33408391780635,  -0.115416570167785,  4.81735395373655 ],
        [-1.33408391780635,   0.115416570167783, -4.81735395373654 ],
        [ 4.81591881210246,   0.202844182067796, -1.32882663769163 ],
        [ 0.164760721921360, -4.99455035545062,  -0.165289598523321],
        [-0.164760721921362,  4.99455035545062,   0.165289598523319],
        [-4.81591881210246,  -0.202844182067798,  1.32882663769163 ],
    ])
)


CONFIG_2D_1: Final = TrussConfig(
    supports=[0, 1],
    move_node=2,
    triangles=[(0,1,2)],
    initial_pos=np.array([
        [0, 0],
        [7.0710678118654675, 0],
        [3.5355339059327338, 6.123724356957938],
    ])
)

CONFIG_2D_2: Final = TrussConfig(
    supports=[0, 3],
    move_node=5,
    triangles=[(0,1,2), (1,3,4), (2,4,5)],
    initial_pos=np.array([
        [0, 0],
        [7.0710678118654675, 0],
        [3.5355339059327338, 6.123724356957938],
        [14.142135623730935, 0],
        [10.6066017177982, 6.123724356957938],
        [7.0710678118654675, 12.247448713915876],
    ])
)
