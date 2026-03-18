import math
from collections.abc import Callable, Generator
from functools import partial
from typing import Final

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import anim
from . import constrain as cstr
from . import grav
from . import tubetruss as tt
from .arraytypes import Matrix, Vector


# Left feet / end-effectors
L1: Final = 0
L2: Final = 1
L3: Final = 2

# Right feet / end effectors
R1: Final = 3
R2: Final = 4
R3: Final = 5

# Left side of chassis / body
P1: Final = 6
P2: Final = 7
P3: Final = 8

# Right side of chassis / body
Q1: Final = 9
Q2: Final = 10
Q3: Final = 11

# Slices
FEET: Final = slice(L1, R3+1)
CHASSIS: Final = slice(P1, Q3+1)

# Links
L1_L2: Final = 0
L2_P3: Final = 1
P3_L1: Final = 2
P1_L2: Final = 3
L2_L3: Final = 4
L3_P1: Final = 5
L1_P2: Final = 6
P2_L3: Final = 7
L3_L1: Final = 8

R1_R2: Final = 9
R2_Q3: Final = 10
Q3_R1: Final = 11
Q1_R2: Final = 12
R2_R3: Final = 13
R3_Q1: Final = 14
R1_Q2: Final = 15
Q2_R3: Final = 16
R3_R1: Final = 17

# Truss structures
LEG_TRUSS: Final = tt.Truss.from_trails(
    # These must follow the link order defined above.
    (L1, L2, P3, L1),
    (P1, L2, L3, P1),
    (L1, P2, L3, L1),
    (R1, R2, Q3, R1),
    (Q1, R2, R3, Q1),
    (R1, Q2, R3, R1),
)
CHASSIS_TRUSS: Final = tt.Truss.from_trails(
    (P1, Q1),
    (P2, Q2),
    (P3, Q3),
    (P1, Q2),
    (P2, Q3),
    (P3, Q1),
    (P1, P2),
    (P2, P3),
    (P3, P1),
    (Q1, Q2),
    (Q2, Q3),
    (Q3, Q1),
)

# Roller setup
CONTROL: Final = tt.LengthControl.from_trails(
    (P3_L1, L1_L2, L2_P3),  # Rollers 01 and 02
    (P1_L2, L2_L3, L3_P1),  # Rollers 03 and 04
    (P2_L3, L3_L1, L1_P2),  # Rollers 05 and 06
    (R2_Q3, R1_R2, Q3_R1),  # Rollers 07 and 08
    (R1_Q2, R3_R1, Q2_R3),  # Rollers 09 and 10
    (R3_Q1, R2_R3, Q1_R2),  # Rollers 11 and 12
    n_static=CHASSIS_TRUSS.n_links,
)

# Point masses
MASS: Final = np.zeros(12)
MASS[FEET] = 1.
MASS[CHASSIS] = 6.

# Constraint points
CL1: Final = cstr.Point.node(L1, 12)
CL2: Final = cstr.Point.node(L2, 12)
CL3: Final = cstr.Point.node(L3, 12)
CR1: Final = cstr.Point.node(R1, 12)
CR2: Final = cstr.Point.node(R2, 12)
CR3: Final = cstr.Point.node(R3, 12)
CP1: Final = cstr.Point.node(P1, 12)
CP2: Final = cstr.Point.node(P2, 12)
CP3: Final = cstr.Point.node(P3, 12)
CQ1: Final = cstr.Point.node(Q1, 12)
CQ2: Final = cstr.Point.node(Q2, 12)
CQ3: Final = cstr.Point.node(Q3, 12)
CCOM: Final = cstr.Point.com(MASS)

# Physical size
# In actuality, we have 1125 * 12 * 4 ticks per side, but we
# increase this to account for not simulating node offsets.
# This factor is a lot more than we would expect. We haven't figured out why.
SLOP_FACTOR = 30
TICKS_PER_SIDE: Final = 1125 * 12 * 4 * SLOP_FACTOR


def make_pos(
    height: float,
    chassis_length: float,
    chassis_angle: float,
    chassis_width: float,
    foot_distance: float,
    *,
    tube_length: float = 3.
) -> Matrix:
    chassis_angle = math.radians(chassis_angle)
    y0 = chassis_width * 0.5
    pos = np.zeros((12, 3))

    # Feet
    fx = foot_distance * 0.5
    fy = math.sqrt(0.25*tube_length**2 - tube_length*fx - height**2)
    pos[L1] = [ fx,  y0+fy, 0.]
    pos[L2] = [-fx,  y0+fy, 0.]
    pos[R1] = [ fx, -y0-fy, 0.]
    pos[R2] = [-fx, -y0-fy, 0.]

    # Chassis
    px = chassis_length * 0.5
    py = px * math.sqrt(3.) * math.sin(chassis_angle)
    pz = px * math.sqrt(3.) * math.cos(chassis_angle) + height
    pos[P1] = [-px,  y0+py, pz]
    pos[P2] = [ px,  y0+py, pz]
    pos[Q1] = [-px, -y0-py, pz]
    pos[Q2] = [ px, -y0-py, pz]
    pos[P3] = [0.,  y0, height]
    pos[Q3] = [0., -y0, height]

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
ROLLING_POS: Final = make_pos(0, 0.5, 0, 1.25, 1.0)


def make_robot(init_pos: Matrix = CRAWLING_POS) -> tt.TrussRobot:
    pos = init_pos.copy()
    truss = LEG_TRUSS.attach(CHASSIS_TRUSS)
    return tt.TrussRobot(pos, truss, CONTROL)


def make_stabilizer(init_pos: Matrix = CRAWLING_POS) -> grav.Stabilizer:
    source_pos = init_pos.copy()
    rel_mass = MASS / np.sum(MASS)
    return grav.Stabilizer(source_pos, rel_mass=rel_mass)


def set_up_animation(
    init_pos: Matrix = CRAWLING_POS,
    *,
    trace_len: int = 100,
) -> tuple[gl.GLViewWidget, Callable[[Matrix], None]]:
    items: list[anim.AnimationItem] = []
    chassis_mesh = gl.GLMeshItem(
        meshdata=gl.MeshData(
            vertexes=init_pos[CHASSIS_TRUSS.nodes],
            faces=[
                [0, 1, 2], [3, 4, 5],
                [0, 3, 5], [0, 2, 5],
                [1, 4, 5], [1, 2, 5],
                [0, 1, 4], [0, 3, 4],
            ],
        ),
        color=pg.mkColor(anim.OKABE_ITO[-1]),
    )
    chassis_mesh.setGLOptions('opaque')
    items.append(anim.BodyMesh(CHASSIS_TRUSS.nodes, chassis_mesh))
    items.append(anim.draw_links(
        CHASSIS_TRUSS.links.ravel(),
        init_pos,
        color='black',
        width=4,
    ))
    triangles = [
        [P1, L2, L3, P1],
        [P2, L3, L1, P2],
        [P3, L1, L2, P3],
        [Q1, R2, R3, Q1],
        [Q2, R3, R1, Q2],
        [Q3, R1, R2, Q3],
    ]
    items += (
        anim.draw_links(trail, init_pos, color=color)
        for trail, color in zip(triangles, anim.OKABE_ITO[1:])
    )
    for trail in triangles:
        marks = gl.GLScatterPlotItem(
            pos=init_pos[trail],
            size=8,
            color=pg.mkColor(anim.OKABE_ITO[0]),
        )
        marks.setGLOptions('opaque')
        markers = anim.Markers(trail, [0.05, 0.95, 1.05, 1.95, 2.05, 2.95], marks)
        markers.update_pos(init_pos)
        items.append(markers)
    items += anim.draw_traces(range(12), trace_len, init_pos)

    view = gl.GLViewWidget()
    view.addItem(gl.GLGridItem())
    anim.add_all_to_view(items, view)
    return view, partial(anim.update_all_pos, items)


def parabolic(k: float, t: float) -> float:
    return 2. * k * (0.5-t)


def adjust_roller(
    robot: tt.TrussRobot,
    roller: int,
    amount: float,
) -> Vector:
    dq = np.zeros(robot.n_rollers)
    dq[roller] = amount
    # We might be able to make better constraints than this.
    constraint = cstr.CompoundConstraint([
        cstr.Motion.make(CL1, x=0, y=0, z=0),
        cstr.Motion.make(CL2, y=0, z=0),
        cstr.Motion.make(CR1, z=0),
    ])
    robot.apply_roll(dq, constraint)
    return dq


def nudge_node(
    robot: tt.TrussRobot,
    node: int,
    x: float,
    y: float,
    z: float,
) -> Vector:
    feet = {L1, L2, R1, R2}
    feet.discard(node)
    motion = cstr.CompoundConstraint([
        cstr.Motion.make(cstr.Point.node(node, robot.n_nodes), x, y, z),
        *(
            cstr.Motion.lock(cstr.Point.node(foot, robot.n_nodes))
            for foot in feet
        ),
    ])
    return robot.take_substep(motion, respect_floor=True)


def nudge_chassis(
    robot: tt.TrussRobot,
    x: float,
    y: float,
    z: float,
) -> Vector:
    chassis_mass = np.zeros(len(robot.pos))
    chassis_mass[CHASSIS] = 1.
    chassis_com = cstr.Point.com(chassis_mass)
    motion = cstr.CompoundConstraint((
        cstr.Motion.make(chassis_com, x, y, z),
        cstr.Motion.make(CP3-CQ3, x=0, z=0),
        cstr.Motion.make(CL1, z=0),
        cstr.Motion.make(CL2, z=0),
        cstr.Motion.make(CR1, z=0),
        cstr.Motion.make(CR2, z=0),
        cstr.Motion.make(cstr.Point.avg(CL1, CL2, CR1, CR2), x=0, y=0),
    ))
    return robot.take_substep(motion)


def crawl(
    robot: tt.TrussRobot,
    cycles: int = 1,
    step_length: tuple[float, float] = (0.125, 0.),
    *,
    resolution: int = 50,
) -> Generator[Vector]:
    chassis_mass = np.zeros(robot.n_nodes)
    chassis_mass[CHASSIS] = 1.
    chassis_com = cstr.Point.com(chassis_mass)
    chassis_up = chassis_com - cstr.Point.avg(CP3, CQ3)
    no_wobble = cstr.Motion(chassis_up, np.eye(3)[0:2], np.zeros(2))
    dx, dy = step_length
    dx /= resolution
    dy /= resolution
    ds = math.hypot(dx, dy)
    steadily_forward = cstr.Motion.make(chassis_com, x=0.25 * dx)
    feet = (CL2, CL1, CR2, CR1)
    for foot in (feet * cycles):
        motion = cstr.CompoundConstraint([
            cstr.Motion.make(foot, x=dx, y=dy, z=partial(parabolic, ds)),
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
    robot: tt.TrussRobot,
    dist: float = 0.6,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    dx = dist / resolution
    constraint = cstr.CompoundConstraint((
        cstr.Motion.make(CP2, dx),
        cstr.Motion.make(CQ2, dx),
        cstr.Motion.lock(CL1),
        cstr.Motion.lock(CR1),
        cstr.Motion.lock(CL2),
        cstr.Motion.lock(CR2),
    ))
    yield from robot.take_step(constraint, resolution=resolution)


def reach(
    robot: tt.TrussRobot,
    dist: float = 1.,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    constraint = cstr.CompoundConstraint((
        cstr.Motion.make(CL3, x=dist / resolution),
        cstr.Motion.make(CR3, x=dist / resolution),
        cstr.Motion.lock(CP3),
        cstr.Motion.lock(CQ3),
        cstr.Motion.lock(CL1),
        cstr.Motion.lock(CR1),
    ))
    yield from robot.take_step(
        constraint,
        resolution=resolution,
        allow_redundant=True,
    )


def roll(
    robot: tt.TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[Vector]:
    chassis_midpoints = (
        cstr.Point.avg(CP1, CQ1),
        cstr.Point.avg(CP3, CQ3),
        cstr.Point.avg(CP2, CQ2),
    )
    foot_pairs = (
        (CL1, CR1),
        (CL3, CR3),
        (CL2, CR2),
    )
    base = chassis_midpoints[i-2]
    face = chassis_midpoints[i-1]
    foot_l, foot_r = foot_pairs[i]
    arm_l, arm_r = foot_pairs[i-2]
    other_feet = [
        foot
        for j, pair in enumerate(foot_pairs)
        for foot in pair
        if j != i
    ]
    chassis_mass = np.zeros(robot.n_nodes)
    chassis_mass[CHASSIS] = 1.
    chassis_com = cstr.Point.com(chassis_mass)
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
    yield from robot.take_step(step_1, resolution=resolution)
    dx = ((face - feet_midpoint).get(robot.pos)[0] - 0.5*0.875) / resolution
    foot_arc = partial(parabolic, -dx)
    step_2 = cstr.CompoundConstraint((
        cstr.Motion.lock(chassis_com),
        cstr.Motion.make(face - base, z=0.),
        cstr.Motion.make(foot_l, x=dx, z=foot_arc),
        cstr.Motion.make(foot_r, x=dx, z=foot_arc),
    ))
    yield from robot.take_step(step_2, resolution=resolution)
    step_3 = cstr.CompoundConstraint((
        cstr.Motion.lock(face),
        cstr.Orbit.about_y(robot.pos, base-face, np.pi/3, resolution),
        cstr.Motion.make(chassis_com - face, y=0.),
        cstr.Motion.make(base - face, y=0.),
        cstr.Motion.lock(foot_l),
        cstr.Motion.lock(foot_r),
        cstr.Orbit.about_y(robot.pos, arm_l-foot_l, np.pi, resolution),
        cstr.Orbit.about_y(robot.pos, arm_r-foot_r, np.pi, resolution),
    ))
    yield from robot.take_step(step_3, resolution=resolution)
