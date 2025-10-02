import numpy as np
from dataclasses import dataclass
from typing import Final, TypeAlias

from linalg import Matrix, Vector

IntMatrix: TypeAlias = np.ndarray[tuple[int, int], np.dtype[np.int64]]
IntVector: TypeAlias = np.ndarray[tuple[int], np.dtype[np.int64]]


@dataclass(kw_only=True)
class CommonData:
    support_move_dict: dict[int, dict[str, IntVector]]
    edge_nodes_dict: dict[int, IntMatrix]
    triangle_nodes_dict: dict[int, IntMatrix]
    initial_node_positions: dict[int, Matrix]


Data3D: Final = CommonData(
    support_move_dict={
        1: {"support": np.array([2, 6, 4]), "move": np.array([1])},
        2: {"support": np.array([2, 6, 4]), "move": np.array([1])}
    },
    edge_nodes_dict={
        1: np.array([[1,3],[3,4],[4,1],[1,5],[5,6],[6,1],[2,3],[3,5],[5,2],[2,4],[4,6],[6,2]]),
        2: np.array([[1,3],[3,5],[5,1],[1,4],[4,6],[6,1],[2,3],[3,4],[4,2],[2,5],[5,6],[6,2]])
    },
    triangle_nodes_dict={
        1: np.array([[1,3,4],[1,5,6],[2,3,5],[2,4,6]]),
        2: np.array([[1,3,5],[1,4,6],[2,3,4],[2,5,6]])
    },
    initial_node_positions={
        1: np.array([
        [ 1.33408391780635, -0.115416570167785,  4.81735395373655],
        [-1.33408391780635,  0.115416570167783, -4.81735395373654],
        [ 4.81591881210246,  0.202844182067796, -1.32882663769163],
        [ 0.164760721921360, -4.99455035545062, -0.165289598523321],
        [-0.164760721921362,  4.99455035545062,  0.165289598523319],
        [-4.81591881210246, -0.202844182067798,  1.32882663769163]]),
        2: np.array([
        [ 1.33408391780635,   -0.115416570167785,  4.81735395373655],
        [-1.33408391780635,    0.115416570167783, -4.81735395373654],
        [ 4.81591881210246,    0.202844182067796, -1.32882663769163],
        [ 0.164760721921360,  -4.99455035545062,  -0.165289598523321],
        [-0.164760721921362,   4.99455035545062,   0.165289598523319],
        [-4.81591881210246,   -0.202844182067798,  1.32882663769163]])
    },
)


Data2D: Final = CommonData(
    support_move_dict={
        1: {"support": np.array([1, 2]), "move": np.array([3])},
        2: {"support": np.array([1, 4]), "move": np.array([6])}
    },
    edge_nodes_dict={
        1: np.array([[1,2],[2,3],[3,1]]),
        2: np.array([[1,2],[2,3],[3,1],[2,4],[4,5],[5,2],[3,5],[5,6],[6,3]])
    },
    triangle_nodes_dict={
        1: np.array([[1,2,3]]),
        2: np.array([[1,2,3],[2,4,5],[3,5,6]])
    },
    initial_node_positions={
        1: np.array([
        [0, 0],
        [7.0710678118654675, 0],
        [3.5355339059327338, 6.123724356957938]]),
        2: np.array([
        [0, 0],
        [7.0710678118654675, 0],
        [3.5355339059327338, 6.123724356957938],
        [14.142135623730935, 0],
        [10.6066017177982, 6.123724356957938],
        [7.0710678118654675, 12.247448713915876]])
    },
)
