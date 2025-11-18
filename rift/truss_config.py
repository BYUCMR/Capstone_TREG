import math
from dataclasses import dataclass, field
from typing import Final, SupportsIndex

import numpy as np
from scipy.optimize import fsolve

from .linalg import Matrix
from .tubetruss import TubeTruss

type Index = SupportsIndex | slice[SupportsIndex]
type Lock = tuple[Index, Index]
type Link = tuple[int, int]
type Triangle = tuple[int, int, int]

bars = TubeTruss.make_bars
tris = TubeTruss.make_tris


def rover_builder(h, P_p, P, theta, w_p, w_f, initial_guess=[12, 12]):
    # height off the ground
    # Perimter of triangle on payload
    # perimter of leg triangle
    # angle of payload size
    # width of payload
    # width of feet

    L_p = P_p / 3
    theta = math.radians(theta)
    h_tb = math.sqrt(((P - w_f) / 2) ** 2 - (w_f / 2) ** 2)

    # ground right
    p0 = [w_f / 2, w_p / 2 + math.sqrt(h_tb ** 2 - h ** 2), 0]
    p1 = [-w_f / 2, w_p / 2 + math.sqrt(h_tb ** 2 - h ** 2), 0]

    # Payload right
    p2 = [0, w_p / 2, h]
    p3 = [L_p / 2, w_p / 2 + L_p * math.sqrt(3) / 2 * math.sin(theta), h + L_p * math.sqrt(3) / 2 * math.cos(theta)]
    p4 = [-L_p / 2, w_p / 2 + L_p * math.sqrt(3) / 2 * math.sin(theta), h + L_p * math.sqrt(3) / 2 * math.cos(theta)]
    #  ground left
    p6 = [w_f / 2, -(w_p / 2 + math.sqrt(h_tb ** 2 - h ** 2)), 0]
    p7 = [-w_f / 2, -(w_p / 2 + math.sqrt(h_tb ** 2 - h ** 2)), 0]
    # payload left
    p8 = [0, -w_p / 2, h]
    p9 = [L_p / 2, -(w_p / 2 + L_p * math.sqrt(3) / 2 * math.sin(theta)), h + L_p * math.sqrt(3) / 2 * math.cos(theta)]
    p10 = [-L_p / 2, -(w_p / 2 + L_p * math.sqrt(3) / 2 * math.sin(theta)),
           h + L_p * math.sqrt(3) / 2 * math.cos(theta)]

    def equations(vars):
        y, z = vars
        eq1 = P - math.dist(p0, p3) - math.dist(p3, (0, y, z)) - math.dist(p0, (0, y, z))
        eq2 = math.dist(p3, (0, y, z)) - math.dist(p0, (0, y, z))
        return [eq1, eq2]

    solution, info, ier, msg = fsolve(equations, initial_guess,full_output=True)
    if ier != 1:
        raise RuntimeError("Fsolve failed to converge, no valid solution. Try different starting configuration.")
    if solution[0] < 0 or solution[1] < 0:
        raise ValueError(
            "Invalid solution. Floating node is at negative position. Please provide a different initial guess.")

    p5 = [0, solution[0], solution[1]]
    p11 = [0, -solution[0], solution[1]]

    rover = TrussConfig(
        keep_level=(2, 8),
        triangles=tris([(0, 1, 2), (0, 3, 5), (1, 4, 5), (6, 7, 8), (6, 9, 11), (7, 10, 11)]),
        payload=bars(
            [(2, 8), (3, 9), (4, 10), (2, 9), (3, 10), (4, 8), (2, 3), (3, 4), (2, 4), (8, 10), (8, 9), (9, 10)]),
        initial_pos=np.array([p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11])
    )
    return rover


@dataclass(slots=True, kw_only=True, frozen=True)
class TrussConfig:
    keep_level: tuple[int, int] | None = None
    payload: TubeTruss = field(default_factory=TubeTruss)
    triangles: TubeTruss = field(default_factory=TubeTruss)
    initial_pos: Matrix


CONFIG_3D_ROVER1: Final = TrussConfig(
    keep_level=(2, 8),
    triangles=tris([(0, 1, 2), (0, 3, 5), (1, 4, 5), (6, 7, 8), (6, 9, 11), (7, 10, 11)]),
    payload=bars([(2, 8), (3, 9), (4, 10), (2, 9), (3, 10), (4, 8), (2, 3), (3, 4), (2, 4), (8, 10), (8, 9), (9, 10)]),
    initial_pos=np.array([
        # Side 1
        [0., 11.794, 0.],  # Ground  ( 0)
        [-18., 11.794, 0.],  # Ground  ( 1)
        [-9., 6.1, 5.53],  # Payload ( 2)
        [-5.536, 6.1, 11.53],  # Payload ( 3)
        [-12.464, 6.1, 11.53],  # Payload ( 4)
        [-9., 19.144, 7.809],  # Float   ( 5)
        # Side 2
        [0., -11.794, 0.],  # Ground  ( 6)
        [-18., -11.794, 0.],  # Ground  ( 7)
        [-9., -6.1, 5.53],  # Payload ( 8)
        [-5.536, -6.1, 11.53],  # Payload ( 9)
        [-12.464, -6.1, 11.53],  # Payload (10)
        [-9., -19.144, 7.809],  # Float   (11)
    ]),
)

CONFIG_3D_1: Final = TrussConfig(
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
    triangles=tris([(0, 1, 2)]),
    initial_pos=np.array([
        [0., 0.],
        [7.07106781, 0.],
        [3.53553391, 6.12372436],
    ]),
)

CONFIG_2D_2: Final = TrussConfig(
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
