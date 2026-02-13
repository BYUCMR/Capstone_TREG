import math
from dataclasses import dataclass, field
from typing import Final, SupportsIndex

import numpy as np

from . import indices as I
from .arraytypes import Matrix, Vector
from .tubetruss import TubeTruss

type Index = SupportsIndex | slice[SupportsIndex]
type Lock = tuple[Index, Index]
type Link = tuple[int, int]
type Triangle = tuple[int, int, int]

bars = TubeTruss.make_bars
tris = TubeTruss.make_tris


def rover_builder(h, P_p, theta, w_p, w_f):
    # height off the ground
    # Perimter of triangle on payload
    # angle of payload size
    # width of payload
    # width of feet

    P = 3.
    theta = math.radians(theta)
    y0 = w_p * 0.5
    pos = np.zeros((12, 3))

    # Feet
    fx = w_f * 0.5
    fy = math.sqrt(0.25*P**2 - P*fx - h**2)
    pos[I.L1] = [ fx,  y0+fy, 0.]
    pos[I.L2] = [-fx,  y0+fy, 0.]
    pos[I.R1] = [ fx, -y0-fy, 0.]
    pos[I.R2] = [-fx, -y0-fy, 0.]

    # Payload
    px = P_p / 6.
    py = px * math.sqrt(3.) * math.sin(theta)
    pz = px * math.sqrt(3.) * math.cos(theta) + h
    pos[I.PL1] = [-px,  y0+py, pz]
    pos[I.PL2] = [ px,  y0+py, pz]
    pos[I.PR1] = [-px, -y0-py, pz]
    pos[I.PR2] = [ px, -y0-py, pz]
    pos[I.PL3] = [0.,  y0, h]
    pos[I.PR3] = [0., -y0, h]

    # Elbows
    # Don't ask me what physical meaning this has.
    # All I know is that it satisfies the length constraint
    # and makes the elbow equidistant from the foot and shoulder.
    dx = fx - px
    dy = fy - py
    k1 = dy**2 + pz**2
    k2 = k1 + dx**2
    k3 = 0.5*(fx**2 - px**2 + fy**2 - py**2 - pz**2)
    k4 = np.sum(np.square(np.cross([px, py, pz], [fx, fy, 0.])))
    k5 = fy*py*k2 - k4 - k3**2 + 0.25*k1*(P-np.sqrt(k2))**2
    elbow_y = (dy*k3 + fy*pz**2 + pz*np.sqrt(k5)) / k1
    elbow_z = (dy * elbow_y - k3) / pz
    pos[I.L3] = [0.,  y0+elbow_y, elbow_z]
    pos[I.R3] = [0., -y0-elbow_y, elbow_z]

    mass = np.zeros(len(pos))
    mass[I.FEET] = 1.
    mass[I.PAYLOAD] = 6.
    return TrussConfig(
        triangles=tris([
            (I.PL3,I.L1,I.L2),
            (I.PL1,I.L2,I.L3),
            (I.PL2,I.L3,I.L1),
            (I.PR3,I.R1,I.R2),
            (I.PR1,I.R2,I.R3),
            (I.PR2,I.R3,I.R1),
        ]),
        payload=bars([
            (I.PL1, I.PR1),
            (I.PL2, I.PR2),
            (I.PL3, I.PR3),
            (I.PL1, I.PR2),
            (I.PL2, I.PR3),
            (I.PL3, I.PR1),
            (I.PL1, I.PL2),
            (I.PL2, I.PL3),
            (I.PL3, I.PL1),
            (I.PR1, I.PR2),
            (I.PR2, I.PR3),
            (I.PR3, I.PR1),
        ]),
        initial_pos=pos,
        mass=mass,
    )

def setup_rover_builder(w_p):
    # height off the ground
    # Perimter of triangle on payload
    # angle of payload size
    # width of payload
    # width of feet

    L_p = 1/2
    L_t = 1
    h_b = math.sqrt(3)/2*L_p
    h_t = math.sqrt(3)/2*L_t

    H = math.sqrt(h_t**2-(1/3*h_b-2/3*h_t)**2)

    pos = np.zeros((12, 3))

    pos[I.PL3] = [ w_p,0,0]
    pos[I.PL2] = [w_p,L_p/2,h_b]
    pos[I.PL1] = [w_p,-L_p/2,h_b]

    pos[I.PR3] = [-w_p,0,0]
    pos[I.PR2] = [-w_p,L_p/2,h_b]
    pos[I.PR1] = [-w_p,-L_p/2,h_b]

    #
    pos[I.L1] = [ w_p+H,L_t/2,0]
    pos[I.L2] = [w_p+H,0,h_t]
    pos[I.L3] = [w_p+H,-L_t/2,0]

    pos[I.R1] = [-w_p-H,L_t/2,0]
    pos[I.R2] = [-w_p-H,0,h_t]
    pos[I.R3] = [-w_p-H,-L_t/2,0]

    mass = np.zeros(len(pos))
    mass[I.FEET] = 1.
    mass[I.PAYLOAD] = 6.
    
    return TrussConfig(
        triangles=tris([
            (I.PL3,I.L1,I.L2),
            (I.PL1,I.L2,I.L3),
            (I.PL2,I.L3,I.L1),
            (I.PR3,I.R1,I.R2),
            (I.PR1,I.R2,I.R3),
            (I.PR2,I.R3,I.R1),
        ]),
        payload=bars([
            (I.PL1, I.PR1),
            (I.PL2, I.PR2),
            (I.PL3, I.PR3),
            (I.PL1, I.PR2),
            (I.PL2, I.PR3),
            (I.PL3, I.PR1),
            (I.PL1, I.PL2),
            (I.PL2, I.PL3),
            (I.PL3, I.PL1),
            (I.PR1, I.PR2),
            (I.PR2, I.PR3),
            (I.PR3, I.PR1),
        ]),
        initial_pos=pos,
        mass=mass,
    )



@dataclass(slots=True, kw_only=True, frozen=True)
class TrussConfig:
    payload: TubeTruss = field(default_factory=TubeTruss)
    triangles: TubeTruss = field(default_factory=TubeTruss)
    initial_pos: Matrix
    mass: Vector | float = 1.


ROVER_CONFIG: Final = rover_builder(0.625, 1.5, 0, 1.25, 0.875)


OLD_ROVER_CONFIG: Final = TrussConfig(
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

