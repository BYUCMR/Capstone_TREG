import math
from collections.abc import Callable, Generator, Iterable
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
from .arraytypes import IndexVector, Matrix, Vector
from .robot import TrussRobot


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
L1_L2: Final = 0
L2_PL3: Final = 1
PL3_L1: Final = 2
PL1_L2: Final = 3
L2_L3: Final = 4
L3_PL1: Final = 5
L1_PL2: Final = 6
PL2_L3: Final = 7
L3_L1: Final = 8

R1_R2: Final = 9
R2_PR3: Final = 10
PR3_R1: Final = 11
PR1_R2: Final = 12
R2_R3: Final = 13
R3_PR1: Final = 14
R1_PR2: Final = 15
PR2_R3: Final = 16
R3_R1: Final = 17

# Truss structures
LEG_INCIDENCE: Final = np.zeros((18, 12), dtype=np.int8)
LEG_INCIDENCE[L1_L2, (L1, L2)] = (1, -1)
LEG_INCIDENCE[L2_PL3, (L2, PL3)] = (1, -1)
LEG_INCIDENCE[PL3_L1, (PL3, L1)] = (1, -1)
LEG_INCIDENCE[PL1_L2, (PL1, L2)] = (1, -1)
LEG_INCIDENCE[L2_L3, (L2, L3)] = (1, -1)
LEG_INCIDENCE[L3_PL1, (L3, PL1)] = (1, -1)
LEG_INCIDENCE[L1_PL2, (L1, PL2)] = (1, -1)
LEG_INCIDENCE[PL2_L3, (PL2, L3)] = (1, -1)
LEG_INCIDENCE[L3_L1, (L3, L1)] = (1, -1)

LEG_INCIDENCE[R1_R2, (R1, R2)] = (1, -1)
LEG_INCIDENCE[R2_PR3, (R2, PR3)] = (1, -1)
LEG_INCIDENCE[PR3_R1, (PR3, R1)] = (1, -1)
LEG_INCIDENCE[PR1_R2, (PR1, R2)] = (1, -1)
LEG_INCIDENCE[R2_R3, (R2, R3)] = (1, -1)
LEG_INCIDENCE[R3_PR1, (R3, PR1)] = (1, -1)
LEG_INCIDENCE[R1_PR2, (R1, PR2)] = (1, -1)
LEG_INCIDENCE[PR2_R3, (PR2, R3)] = (1, -1)
LEG_INCIDENCE[R3_R1, (R3, R1)] = (1, -1)

LEG_TRUSS: Final = tt.Truss(LEG_INCIDENCE)
PAYLOAD_TRUSS: Final = tt.Truss.from_trails(
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
)

# Roller setup
ROLL_TO_LENGTH: Final = np.zeros((30, 12), dtype=np.intp)
ROLL_TO_LENGTH[(L1_L2, PL3_L1), 0] = (-1, 1)
ROLL_TO_LENGTH[(L1_L2, L2_PL3), 1] = (1, -1)
ROLL_TO_LENGTH[(L2_L3, PL1_L2), 2] = (-1, 1)
ROLL_TO_LENGTH[(L2_L3, L3_PL1), 3] = (1, -1)
ROLL_TO_LENGTH[(L3_L1, PL2_L3), 4] = (-1, 1)
ROLL_TO_LENGTH[(L3_L1, L1_PL2), 5] = (1, -1)
ROLL_TO_LENGTH[(R1_R2, PR3_R1), 7] = (1, -1)
ROLL_TO_LENGTH[(R1_R2, R2_PR3), 6] = (-1, 1)
ROLL_TO_LENGTH[(R2_R3, PR1_R2),11] = (1, -1)
ROLL_TO_LENGTH[(R2_R3, R3_PR1),10] = (-1, 1)
ROLL_TO_LENGTH[(R3_R1, PR2_R3), 9] = (1, -1)
ROLL_TO_LENGTH[(R3_R1, R1_PR2), 8] = (-1, 1)

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

# Physical size
TICKS_PER_SIDE: Final = 1125 * 12 * 6


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


def make_pos_sitting(
    payload_length: float,
    payload_angle: float,
    payload_width: float,
    foot_distance: float,
    *,
    tube_length: float = 3.
) -> Matrix:
    # Perimter of triangle on payload
    # angle of payload size
    # width of payload
    # width of feet

    payload_angle = math.radians(payload_angle)
    y0 = payload_width * 0.5
    pos = np.zeros((12, 3))

    # Feet
    fx = foot_distance * 0.5
    fy = math.sqrt(0.25*tube_length**2 - tube_length*fx)
    ez = fx * math.sqrt(0.75*tube_length**2 + tube_length*fx) / (0.5*tube_length + fx)
    pos[L1] = [ fx,  y0+fy, 0.]
    pos[L2] = [-fx,  y0+fy, 0.]
    pos[R1] = [ fx, -y0-fy, 0.]
    pos[R2] = [-fx, -y0-fy, 0.]
    pos[L3] = [ 0.,  y0+fy, ez]
    pos[R3] = [ 0., -y0-fy, ez]

    # Payload
    px = payload_length * 0.5
    py = px * math.sqrt(3.) * math.sin(payload_angle)
    pz = px * math.sqrt(3.) * math.cos(payload_angle)
    pos[PL1] = [-px,  y0+py, pz]
    pos[PL2] = [ px,  y0+py, pz]
    pos[PR1] = [-px, -y0-py, pz]
    pos[PR2] = [ px, -y0-py, pz]
    pos[PL3] = [0.,  y0, 0.]
    pos[PR3] = [0., -y0, 0.]

    return pos


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

    pos[PL3] = [0,w_p,0]
    pos[PL2] = [L_p/2,w_p,h_b]
    pos[PL1] = [-L_p/2,w_p,h_b]

    pos[PR3] = [0,-w_p,0]
    pos[PR2] = [L_p/2,-w_p,h_b]
    pos[PR1] = [-L_p/2,-w_p,h_b]

    #
    pos[L1] = [L_t/2,w_p+H,0]
    pos[L2] = [-L_t/2,w_p+H,0]
    pos[L3] = [0,w_p+H,h_t]

    pos[R1] = [L_t/2,-w_p-H,0]
    pos[R2] = [-L_t/2,-w_p-H,0]
    pos[R3] = [0,-w_p-H,h_t]

    mass = np.zeros(len(pos))
    mass[FEET] = 1.
    mass[PAYLOAD] = 6.

    return pos

CRAWLING_POS: Final = make_pos(0.625, 0.5, 0, 1.25, 0.875)
ROLLING_POS: Final = make_pos(0, 0.5, 0, 1.25, 0.875)


def make_robot(init_pos: Matrix = CRAWLING_POS) -> TrussRobot:
    pos = init_pos.copy()
    truss = LEG_TRUSS.attach(PAYLOAD_TRUSS)
    control = tt.LengthControl.from_forward(ROLL_TO_LENGTH)
    return TrussRobot(pos, truss, control)


def make_stabilizer(init_pos: Matrix = CRAWLING_POS) -> grav.Stabilizer:
    source_pos = init_pos.copy()
    rel_mass = MASS / np.sum(MASS)
    return grav.Stabilizer(source_pos, rel_mass=rel_mass)


def draw_payload_bars(payload: tt.Truss, pos: Matrix) -> anim.DrawnLinks:
    return anim.draw_links(payload.links.ravel(), pos, color='black', width=4)


def draw_payload_mesh(payload: tt.Truss, pos: Matrix) -> anim.BodyMesh:
    payload_faces = [[0, 1, 2], [3, 4, 5],
                     [0, 3, 5], [0, 2, 5],
                     [1, 4, 5], [1, 2, 5],
                     [0, 1, 4], [0, 3, 4]]

    meshdata = gl.MeshData(
        vertexes=pos[payload.nodes],
        faces=payload_faces,
    )
    mesh = gl.GLMeshItem(
        meshdata=meshdata,
        color=pg.mkColor(anim.OKABE_ITO[-1]),
    )
    mesh.setGLOptions('opaque')
    return anim.BodyMesh(payload.nodes, mesh)


def draw_triangles(truss: tt.Truss, pos: Matrix) -> list[anim.DrawnLinks]:
    drawn_tubes: list[anim.DrawnLinks] = []
    for i, j in enumerate(range(0, truss.n_links, 3)):
        nodes = truss.links[j:j+3].ravel()
        color = anim.OKABE_ITO[i % (len(anim.OKABE_ITO) - 1) + 1]
        drawn_tube = anim.draw_links(nodes, pos, color=color)
        drawn_tubes.append(drawn_tube)
    return drawn_tubes


def draw_markers(trails: Iterable[IndexVector], pos: Matrix) -> list[anim.Markers]:
    all_markers: list[anim.Markers] = []
    for trail in trails:
        marks = gl.GLScatterPlotItem(
            pos=pos[trail],
            size=8,
            color=pg.mkColor(anim.OKABE_ITO[0]),
        )
        marks.setGLOptions('opaque')
        markers = anim.Markers(trail, [0.05, 0.95, 1.05, 1.95, 2.05, 2.95], marks)
        all_markers.append(markers)
    return all_markers


def set_up_animation(
    init_pos: Matrix = CRAWLING_POS,
    *,
    trace_len: int = 100,
) -> tuple[gl.GLViewWidget, Callable[[Matrix], None]]:
    view = gl.GLViewWidget()
    view.addItem(gl.GLGridItem())
    payload_mesh = draw_payload_mesh(PAYLOAD_TRUSS, init_pos)
    payload_bars = draw_payload_bars(PAYLOAD_TRUSS, init_pos)
    triangles = draw_triangles(LEG_TRUSS, init_pos)
    traces = anim.draw_traces(range(12), trace_len, init_pos)
    markers = draw_markers(
        [
            [PL1, L2, L3, PL1],
            [PL2, L3, L1, PL2],
            [PL3, L1, L2, PL3],
            [PR1, R2, R3, PR1],
            [PR2, R3, R1, PR2],
            [PR3, R1, R2, PR3],
        ],
        init_pos,
    )
    items = [payload_mesh, payload_bars, *triangles, *traces, *markers]
    anim.add_all_to_view(items, view)
    return view, partial(anim.update_all_pos, items)


def crawl(
    robot: TrussRobot,
    cycles: int = 1,
    step_length: tuple[float, float] = (0.125, 0.),
    *,
    resolution: int = 50,
) -> Generator[Vector]:
    payload_mass = np.zeros(robot.n_nodes)
    payload_mass[PAYLOAD] = 1.
    payload_com = cstr.Point.com(payload_mass)
    payload_up = payload_com - cstr.Point.avg(CPL3, CPR3)
    no_wobble = cstr.Motion(payload_up, np.eye(3)[0:2], np.zeros(2))
    dx, dy = step_length
    dx /= resolution
    dy /= resolution
    ds = math.hypot(dx, dy)
    steadily_forward = cstr.Motion.make(payload_com, x=0.25 * dx)
    feet = (CL2, CL1, CR2, CR1)
    for foot in (feet * cycles):
        motion = cstr.CompoundConstraint([
            cstr.Motion.make(foot, x=dx, y=dy, z=partial(steps.parabolic, ds)),
            *(
                cstr.Motion.lock(other_foot)
                for other_foot in feet
                if foot is not other_foot
            ),
            steadily_forward,
            no_wobble,
        ])
        yield from robot.take_step(motion, resolution=resolution)


def lean(
    robot: TrussRobot,
    dist: float = 0.6,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    dx = dist / resolution
    constraint = cstr.CompoundConstraint((
        cstr.Motion.make(CPL2, dx),
        cstr.Motion.make(CPR2, dx),
        cstr.Motion.lock(CL1),
        cstr.Motion.lock(CR1),
        cstr.Motion.lock(CL2),
        cstr.Motion.lock(CR2),
    ))
    yield from robot.take_step(constraint, resolution=resolution)


def reach(
    robot: TrussRobot,
    dist: float = 1.,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    constraint = cstr.CompoundConstraint((
        cstr.Motion.make(CL3, x=dist / resolution),
        cstr.Motion.make(CR3, x=dist / resolution),
        cstr.Motion.lock(CPL3),
        cstr.Motion.lock(CPR3),
        cstr.Motion.lock(CL1),
        cstr.Motion.lock(CR1),
    ))
    yield from robot.take_step(
        constraint,
        resolution=resolution,
        allow_redundant=True,
    )


def roll(
    robot: TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[Vector]:
    payload_midpoints = (
        cstr.Point.avg(CPL1, CPR1),
        cstr.Point.avg(CPL3, CPR3),
        cstr.Point.avg(CPL2, CPR2),
    )
    foot_pairs = (
        (CL1, CR1),
        (CL3, CR3),
        (CL2, CR2),
    )
    base = payload_midpoints[i-2]
    face = payload_midpoints[i-1]
    foot_l, foot_r = foot_pairs[i]
    arm_l, arm_r = foot_pairs[i-2]
    other_feet = [
        foot
        for j, pair in enumerate(foot_pairs)
        for foot in pair
        if j != i
    ]
    payload_mass = np.zeros(robot.n_nodes)
    payload_mass[PAYLOAD] = 1.
    payload_com = cstr.Point.com(payload_mass)
    feet_midpoint = cstr.Point.avg(foot_l, foot_r)
    step_1 = cstr.CompoundConstraint((
        cstr.Motion.lock(base),
        cstr.Orbit.about_y(robot.pos, face-base, np.pi, resolution),
        cstr.Motion.make(face - base, y=0.),
        cstr.Motion.lock(foot_l),
        cstr.Motion.lock(foot_r),
        *(
            cstr.Motion.make(foot, y=0.)
            for foot in other_feet
        ),
    ))
    yield from robot.take_step(step_1, resolution=resolution, allow_redundant=True)
    dx = ((face - feet_midpoint).get(robot.pos)[0] - 0.5*0.875) / resolution
    foot_arc = partial(steps.parabolic, -dx)
    step_2 = cstr.CompoundConstraint((
        cstr.Motion.lock(payload_com),
        cstr.Motion.make(face - base, z=0.),
        cstr.Motion.make(foot_l, x=dx, z=foot_arc),
        cstr.Motion.make(foot_r, x=dx, z=foot_arc),
    ))
    yield from robot.take_step(step_2, resolution=resolution)
    step_3 = cstr.CompoundConstraint((
        cstr.Motion.lock(face),
        cstr.Orbit.about_y(robot.pos, base-face, np.pi/3, resolution),
        cstr.Motion.make(payload_com - face, y=0.),
        cstr.Motion.make(base - face, y=0.),
        cstr.Motion.lock(foot_l),
        cstr.Motion.lock(foot_r),
        cstr.Orbit.about_y(robot.pos, arm_l-foot_l, np.pi, resolution),
        cstr.Orbit.about_y(robot.pos, arm_r-foot_r, np.pi, resolution),
    ))
    yield from robot.take_step(step_3, resolution=resolution)


def take_command(
    robot: TrussRobot,
    command: steps.Command,
    *,
    resolution: int,
) -> Generator[Vector]:
    if command.x ==0 and command.y == 0 and command.z == 0:
        return
    elif command.mode is steps.Mode.crawling:
        x = command.x * 0.125
        y = -command.y * 0.125
        yield from crawl(robot, 1, (x, y), resolution=resolution)
    elif command.mode is steps.Mode.node_control:
        feet = {L1, L2, R1, R2}
        feet.discard(command.item)
        motion = cstr.CompoundConstraint([
            cstr.Motion.make(
                cstr.Point.node(command.item, robot.n_nodes),
                x=command.x * 0.0005,
                y=command.y * 0.0005,
                z=command.z * 0.0005,
            ),
            *(
                cstr.Motion.lock(cstr.Point.node(foot, robot.n_nodes))
                for foot in feet
            ),
        ])
        yield from robot.take_step(motion, resolution=resolution, respect_floor=True)
