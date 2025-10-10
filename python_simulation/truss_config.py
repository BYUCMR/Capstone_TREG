import numpy as np
from dataclasses import dataclass
from typing import Final, TypeAlias

from linalg import Matrix

IntMatrix: TypeAlias = np.ndarray[tuple[int, int], np.dtype[np.int64]]
IntVector: TypeAlias = np.ndarray[tuple[int], np.dtype[np.int64]]


@dataclass(kw_only=True)
class TrussConfig:
    supports: IntVector
    move_node: int
    edges: IntMatrix
    triangles: IntMatrix
    initial_pos: Matrix


CONFIG_3D_1: Final = TrussConfig(
    supports=np.array([1, 5, 3]),
    move_node=0,
    edges=np.array([[0,2],[2,3],[3,0],[0,4],[4,5],[5,0],[1,2],[2,4],[4,1],[1,3],[3,5],[5,1]]),
    triangles=np.array([[0,2,3],[0,4,5],[1,2,4],[1,3,5]]),
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
    supports=np.array([1, 5, 3]),
    move_node=0,
    edges=np.array([[0,2],[2,4],[4,0],[0,3],[3,5],[5,0],[1,2],[2,3],[3,1],[1,4],[4,5],[5,1]]),
    triangles=np.array([[0,2,4],[0,3,5],[1,2,3],[1,4,5]]),
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
    supports=np.array([0, 1]),
    move_node=2,
    edges=np.array([[0,1],[1,2],[2,0]]),
    triangles=np.array([[0,1,2]]),
    initial_pos=np.array([
        [0, 0],
        [7.0710678118654675, 0],
        [3.5355339059327338, 6.123724356957938],
    ])
)

CONFIG_2D_2: Final = TrussConfig(
    supports=np.array([0, 3]),
    move_node=5,
    edges=np.array([[0,1],[1,2],[2,0],[1,3],[3,4],[4,1],[2,4],[4,5],[5,2]]),
    triangles=np.array([[0,1,2],[1,3,4],[2,4,5]]),
    initial_pos=np.array([
        [0, 0],
        [7.0710678118654675, 0],
        [3.5355339059327338, 6.123724356957938],
        [14.142135623730935, 0],
        [10.6066017177982, 6.123724356957938],
        [7.0710678118654675, 12.247448713915876],
    ])
)
