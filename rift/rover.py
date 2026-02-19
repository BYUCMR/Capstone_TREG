import math
from collections.abc import Generator
from functools import partial
from typing import Final

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import anim
from . import constrain as cstr
from . import grav
from . import steps
from . import tubetruss as tt
from .arraytypes import Matrix, Vector
from .robot import RobotInverse


# Left feet / end-effectors
L1: Final = 0
L2: Final = 1
L3: Final = 2

# Right feet / end effectors
R1: Final = 3
R2: Final = 4
R3: Final = 5

# Left side of payload / body
PL1: Final = 6
PL2: Final = 7
PL3: Final = 8

# Right side of payload / body
PR1: Final = 9
PR2: Final = 10
PR3: Final = 11

# Slices
FEET: Final = slice(L1, R3+1)
PAYLOAD: Final = slice(PL1, PR3+1)

# Links
L1_L2: Final = 1
L2_L3: Final = 4
L3_L1: Final = 7
L1_PL2: Final = 8
L1_PL3: Final = 0
L2_PL3: Final = 2
L2_PL1: Final = 3
L3_PL1: Final = 5
L3_PL2: Final = 6

R1_R2: Final = 10
R2_R3: Final = 13
R3_R1: Final = 16
R1_PR2: Final = 17
R1_PR3: Final = 9
R2_PR3: Final = 11
R2_PR1: Final = 12
R3_PR1: Final = 14
R3_PR2: Final = 15

# Truss structures
LEG_STRUCTURE: Final = tt.TubeTruss.make_tris([
    (PL3, L1, L2),
    (PL1, L2, L3),
    (PL2, L3, L1),
    (PR3, R1, R2),
    (PR1, R2, R3),
    (PR2, R3, R1),
])

PAYLOAD_STRUCTURE: Final = tt.TubeTruss.make_bars([
    (PL1, PR1),
    (PL2, PR2),
    (PL3, PR3),
    (PL1, PR2),
    (PL2, PR3),
    (PL3, PR1),
    (PL1, PL2),
    (PL2, PL3),
    (PL3, PL1),
    (PR1, PR2),
    (PR2, PR3),
    (PR3, PR1),
])

# Roller setup
CONTROL: Final = np.zeros((18, 30), dtype=np.intp)
CONTROL[(L1_L2, L1_PL3), 0] = (-1, 1)
CONTROL[(L1_L2, L2_PL3), 1] = (1, -1)
CONTROL[(L2_L3, L2_PL1), 2] = (-1, 1)
CONTROL[(L2_L3, L3_PL1), 3] = (1, -1)
CONTROL[(L3_L1, L3_PL2), 4] = (-1, 1)
CONTROL[(L3_L1, L1_PL2), 5] = (1, -1)
CONTROL[(R1_R2, R1_PR3), 7] = (1, -1)
CONTROL[(R1_R2, R2_PR3), 6] = (-1, 1)
CONTROL[(R2_R3, R2_PR1),11] = (1, -1)
CONTROL[(R2_R3, R3_PR1),10] = (-1, 1)
CONTROL[(R3_R1, R3_PR2), 9] = (1, -1)
CONTROL[(R3_R1, R1_PR2), 8] = (-1, 1)

# Point masses
MASS: Final = np.zeros(12)
MASS[FEET] = 1.
MASS[PAYLOAD] = 6.

# Constraint points
CL1: Final = cstr.Point.node(L1, 12)
CL2: Final = cstr.Point.node(L2, 12)
CL3: Final = cstr.Point.node(L3, 12)
CR1: Final = cstr.Point.node(R1, 12)
CR2: Final = cstr.Point.node(R2, 12)
CR3: Final = cstr.Point.node(R3, 12)
CPL1: Final = cstr.Point.node(PL1, 12)
CPL2: Final = cstr.Point.node(PL2, 12)
CPL3: Final = cstr.Point.node(PL3, 12)
CPR1: Final = cstr.Point.node(PR1, 12)
CPR2: Final = cstr.Point.node(PR2, 12)
CPR3: Final = cstr.Point.node(PR3, 12)
CCOM: Final = cstr.Point.com(MASS)


def make_pos(
    height: float,
    payload_length: float,
    payload_angle: float,
    payload_width: float,
    foot_distance: float,
    *,
    tube_length: float = 3.
) -> Matrix:
    payload_angle = math.radians(payload_angle)
    y0 = payload_width * 0.5
    pos = np.zeros((12, 3))

    # Feet
    fx = foot_distance * 0.5
    fy = math.sqrt(0.25*tube_length**2 - tube_length*fx - height**2)
    pos[L1] = [ fx,  y0+fy, 0.]
    pos[L2] = [-fx,  y0+fy, 0.]
    pos[R1] = [ fx, -y0-fy, 0.]
    pos[R2] = [-fx, -y0-fy, 0.]

    # Payload
    px = payload_length * 0.5
    py = px * math.sqrt(3.) * math.sin(payload_angle)
    pz = px * math.sqrt(3.) * math.cos(payload_angle) + height
    pos[PL1] = [-px,  y0+py, pz]
    pos[PL2] = [ px,  y0+py, pz]
    pos[PR1] = [-px, -y0-py, pz]
    pos[PR2] = [ px, -y0-py, pz]
    pos[PL3] = [0.,  y0, height]
    pos[PR3] = [0., -y0, height]

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
    k5 = fy*py*k2 - k4 - k3**2 + 0.25*k1*(tube_length-np.sqrt(k2))**2
    elbow_y = (dy*k3 + fy*pz**2 + pz*np.sqrt(k5)) / k1
    elbow_z = (dy * elbow_y - k3) / pz
    pos[L3] = [0.,  y0+elbow_y, elbow_z]
    pos[R3] = [0., -y0-elbow_y, elbow_z]

    return pos


CRAWLING_POS: Final = make_pos(0.625, 0.5, 0, 1.25, 0.875)


def make_robot(init_pos: Matrix = CRAWLING_POS) -> RobotInverse:
    structure = LEG_STRUCTURE + PAYLOAD_STRUCTURE
    pos = init_pos.copy()
    return RobotInverse(structure, CONTROL, pos)


def make_stabilizer(init_pos: Matrix = CRAWLING_POS) -> grav.Stabilizer:
    source_pos = init_pos.copy()
    rel_mass = MASS / np.sum(MASS)
    return grav.Stabilizer(source_pos, rel_mass=rel_mass)


def draw_payload_bars(payload: tt.TubeTruss, pos: Matrix) -> anim.DrawnLinks:
    return anim.draw_links(payload.links, pos, color='black', width=4)


def draw_payload_mesh(payload: tt.TubeTruss, pos: Matrix) -> anim.BodyMesh:
    nodes = sorted(set(payload.nodes))
    payload_faces = [[0, 1, 2], [3, 4, 5],
                     [0, 3, 5], [0, 2, 5],
                     [1, 4, 5], [1, 2, 5],
                     [0, 1, 4], [0, 3, 4]]

    meshdata = gl.MeshData(
        vertexes=pos[nodes],
        faces=payload_faces,
    )
    mesh = gl.GLMeshItem(
        meshdata=meshdata,
        color=pg.mkColor(anim.OKABE_ITO[-1]),
    )
    mesh.setGLOptions('opaque')
    return anim.BodyMesh(nodes, mesh)


def draw_triangles(triangles: tt.TubeTruss, pos: Matrix) -> list[anim.DrawnLinks]:
    drawn_tubes: list[anim.DrawnLinks] = []
    for i, tube in enumerate(triangles):
        color = anim.OKABE_ITO[i % (len(anim.OKABE_ITO) - 1) + 1]
        drawn_tube = anim.draw_links(tube.links, pos, color=color)
        drawn_tubes.append(drawn_tube)
    return drawn_tubes


def make_animator(init_pos: Matrix = CRAWLING_POS) -> anim.Animator:
    view = gl.GLViewWidget()
    view.addItem(gl.GLGridItem())
    payload_mesh = draw_payload_mesh(PAYLOAD_STRUCTURE, init_pos)
    payload_bars = draw_payload_bars(PAYLOAD_STRUCTURE, init_pos)
    triangles = draw_triangles(LEG_STRUCTURE, init_pos)
    traces = anim.draw_traces((L1, L2, R1, R2), init_pos)
    items = [payload_mesh, payload_bars, *triangles, *traces]
    for item in items:
        item.add_to_view(view)
    return anim.Animator(view, items)


def crawl(
    robot: RobotInverse,
    cycles: int = 1,
    step_length: float = 0.125,
    *,
    resolution: int = 50,
) -> Generator[tuple[Matrix, Vector]]:
    payload_mass = np.zeros(len(robot.pos))
    payload_mass[PAYLOAD] = 1.
    payload_com = cstr.Point.com(payload_mass)
    payload_up = payload_com - cstr.Point.avg(CPL3, CPR3)
    no_wobble = cstr.Motion(payload_up, np.eye(3)[0:2], np.zeros(2))
    dx = step_length / resolution
    steadily_forward = cstr.Motion.make(payload_com, x=0.25 * dx)
    feet = (CL2, CL1, CR2, CR1)
    for foot in (feet * cycles):
        motion = cstr.CompoundConstraint([
            cstr.Motion.make(foot, x=dx, y=0., z=partial(steps.parabolic, dx)),
            *(
                cstr.Motion.lock(other_foot)
                for other_foot in feet
                if foot is not other_foot
            ),
            steadily_forward,
            no_wobble,
        ])
        yield from robot.take_step(motion, resolution=resolution)
