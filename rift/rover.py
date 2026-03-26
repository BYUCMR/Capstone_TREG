import enum
import math
from collections.abc import Callable, Generator
from functools import partial
from typing import Final, SupportsIndex

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import anim
from . import grav
from . import tubetruss as tt
from .arraytypes import Matrix, Vector
from .tubetruss import constrain as cstr


@enum.global_enum
class Node(enum.Enum):
    # Left feet / end-effectors
    L1 = 0
    L2 = 1
    L3 = 2
    # Right feet / end effectors
    R1 = 3
    R2 = 4
    R3 = 5
    # Left side of chassis / body
    P1 = 6
    P2 = 7
    P3 = 8
    # Right side of chassis / body
    Q1 = 9
    Q2 = 10
    Q3 = 11

    def __index__(self) -> int:
        return self.value

    def __array__(self) -> Vector:
        out = np.zeros(len(type(self)))
        out[self.value] = 1
        return out

    def __sub__(self, other: cstr.Point) -> Vector:
        return np.array(self) - np.array(other)

    @property
    def i(self) -> int:
        return self.value


# `enum.global_enum` does this, but tools can't tell.
L1: Final = Node.L1
L2: Final = Node.L2
L3: Final = Node.L3
R1: Final = Node.R1
R2: Final = Node.R2
R3: Final = Node.R3
P1: Final = Node.P1
P2: Final = Node.P2
P3: Final = Node.P3
Q1: Final = Node.Q1
Q2: Final = Node.Q2
Q3: Final = Node.Q3


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
TRUSS: Final = tt.Truss.from_trails(
    # These must follow the link order defined above.
    (L1, L2, P3, L1),
    (P1, L2, L3, P1),
    (L1, P2, L3, L1),
    (R1, R2, Q3, R1),
    (Q1, R2, R3, Q1),
    (R1, Q2, R3, R1),
    # These can be in any order.
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
    n_static=12,
)

# Point masses
MASS: Final = np.zeros(12)
CHASSIS_MASS: Final = np.zeros(12)
MASS[:6] = 2.
MASS[6:] = CHASSIS_MASS[6:] = 1.

# Physical size
# In actuality, we have 1125 * 12 * 4 ticks per side, but we
# increase this to account for not simulating node offsets.
TICKS_PER_SIDE: Final = 1125 * 12 * 4 * 1.5


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
    return tt.TrussRobot(pos, TRUSS, CONTROL)


def make_stabilizer(init_pos: Matrix = CRAWLING_POS) -> grav.Stabilizer:
    source_pos = init_pos.copy()
    rel_mass = MASS / np.sum(MASS)
    return grav.Stabilizer(source_pos, rel_mass=rel_mass)


def set_up_animation(
    init_pos: Matrix = CRAWLING_POS,
    *,
    trace_len: SupportsIndex = 100,
) -> tuple[gl.GLViewWidget, Callable[[Matrix], None]]:
    items: list[anim.AnimationItem] = []
    chassis_mesh = gl.GLMeshItem(
        meshdata=gl.MeshData(
            vertexes=init_pos[6:12],
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
    items.append(anim.BodyMesh(range(6, 12), chassis_mesh))
    items.append(anim.draw_links(
        TRUSS.get_links()[18:].ravel(),
        init_pos,
        color='black',
        width=4,
        mode='lines',
    ))
    triangles = [
        [P1.i, L2.i, L3.i, P1.i],
        [P2.i, L3.i, L1.i, P2.i],
        [P3.i, L1.i, L2.i, P3.i],
        [Q1.i, R2.i, R3.i, Q1.i],
        [Q2.i, R3.i, R1.i, Q2.i],
        [Q3.i, R1.i, R2.i, Q3.i],
    ]
    items += (
        anim.draw_links(trail, init_pos, color=color, width=6)
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
    items += anim.draw_traces(range(12), trace_len, init_pos, size=4)

    view = gl.GLViewWidget()
    view.addItem(gl.GLGridItem())
    anim.add_all_to_view(items, view)
    return view, partial(anim.update_all_pos, items)


def adjust_roller(
    robot: tt.TrussRobot,
    roller: SupportsIndex,
    amount: float,
) -> Vector:
    dq = np.zeros(robot.n_rollers)
    dq[roller] = amount
    # We might be able to make better constraints than this.
    constraint = cstr.Static.combine(
        cstr.xyz(L1, x=0, y=0, z=0),
        cstr.xyz(L2, y=0, z=0),
        cstr.xyz(R1, z=0),
    )
    robot.apply_roll(dq, constraint)
    return dq


def nudge_node(
    robot: tt.TrussRobot,
    node: SupportsIndex,
    x: float,
    y: float,
    z: float,
) -> Vector:
    locked = robot.pos[:, 2] < grav.DEFAULT_TOL
    locked[node] = False
    motion = cstr.Static.combine(
        cstr.xyz(Node(node), x, y, z),
        *(
            cstr.lock(Node(n))
            for n in np.flatnonzero(locked)
        ),
    )
    return robot.take_step(motion, respect_floor=True, allow_redundant=True)


def nudge_chassis(
    robot: tt.TrussRobot,
    x: float,
    y: float,
    z: float,
) -> Vector:
    chassis_com = CHASSIS_MASS / np.sum(CHASSIS_MASS)
    motion = cstr.Static.combine(
        cstr.xyz(chassis_com, x, y, z),
        cstr.xyz(P3-Q3, x=0, z=0),
        cstr.xyz(L1, z=0),
        cstr.xyz(L2, z=0),
        cstr.xyz(R1, z=0),
        cstr.xyz(R2, z=0),
        cstr.xyz(cstr.centroid(L1, L2, R1, R2), x=0, y=0),
    )
    return robot.take_step(motion)


def tilt_chassis(
    robot: tt.TrussRobot,
    angle: float,
) -> Vector:
    base = cstr.centroid(P3, Q3)
    face = cstr.centroid(P2, Q2)
    motion = cstr.CompoundConstraint((
        cstr.lock(base),
        cstr.Orbit(face-base, cstr.Y, angle),
        cstr.xyz(face - base, y=0.),
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.xyz(L2, y=0.),
        cstr.xyz(R2, y=0.),
        cstr.xyz(L3, y=0.),
        cstr.xyz(R3, y=0.),
    ))
    return robot.take_step(motion, respect_floor=False)


def crawl(
    robot: tt.TrussRobot,
    cycles: int = 1,
    step_length: tuple[float, float] = (0.125, 0.),
    *,
    resolution: int = 50,
) -> Generator[Vector]:
    chassis_com = CHASSIS_MASS / np.sum(CHASSIS_MASS)
    chassis_up = chassis_com - cstr.centroid(P3, Q3)
    no_wobble = cstr.motion(chassis_up, np.eye(3)[0:2], np.zeros(2))
    x_dist, y_dist = step_length
    steadily_forward = cstr.xyz(
        chassis_com, x=0.25 * x_dist / resolution
    )
    feet = (L2, L1, R2, R1)
    for foot in (feet * cycles):
        motion = cstr.CompoundConstraint([
            cstr.ParabolicPath.make(
                point=foot,
                init_pos=robot.pos,
                delta_x=x_dist,
                delta_y=y_dist,
                resolution=resolution,
            ),
            *(
                cstr.lock(other_foot)
                for other_foot in feet
                if foot is not other_foot
            ),
            steadily_forward,
            no_wobble,
        ])
        yield from robot.repeat_step(motion, times=resolution)


def lean(
    robot: tt.TrussRobot,
    dist: float = 0.6,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    dx = dist / resolution
    constraint = cstr.Static.combine(
        cstr.xyz(P2, dx),
        cstr.xyz(Q2, dx),
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.lock(L2),
        cstr.lock(R2),
    )
    yield from robot.repeat_step(constraint, times=resolution)


def reach(
    robot: tt.TrussRobot,
    dist: float = 1.,
    *,
    resolution: int = 100,
) -> Generator[Vector]:
    constraint = cstr.Static.combine(
        cstr.xyz(L3, x=dist / resolution),
        cstr.xyz(R3, x=dist / resolution),
        cstr.lock(P3),
        cstr.lock(Q3),
        cstr.lock(L1),
        cstr.lock(R1),
    )
    yield from robot.repeat_step(
        constraint,
        times=resolution,
        allow_redundant=True,
    )


def roll(
    robot: tt.TrussRobot,
    *,
    i: int = 0,
    resolution: int = 100,
) -> Generator[Vector]:
    chassis_midpoints = (
        cstr.centroid(P1, Q1),
        cstr.centroid(P3, Q3),
        cstr.centroid(P2, Q2),
    )
    foot_pairs = (
        (L1, R1),
        (L3, R3),
        (L2, R2),
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
    chassis_com = CHASSIS_MASS / np.sum(CHASSIS_MASS)
    feet_midpoint = cstr.centroid(foot_l, foot_r)
    step_1 = cstr.CompoundConstraint((
        cstr.lock(base),
        cstr.Orbit.align(face-base, cstr.X, init_pos=robot.pos, resolution=resolution),
        cstr.xyz(face - base, y=0.),
        cstr.lock(foot_l),
        cstr.lock(foot_r),
        *(
            cstr.xyz(foot, y=0.)
            for foot in other_feet
        ),
    ))
    yield from robot.repeat_step(step_1, times=resolution)
    x_dist = ((face - feet_midpoint) @ robot.pos)[0] - 0.5*0.875
    step_2 = cstr.CompoundConstraint((
        cstr.lock(chassis_com),
        cstr.xyz(face - base, z=0.),
        cstr.ParabolicPath.make(
            foot_l,
            init_pos=robot.pos,
            delta_x=x_dist,
            resolution=resolution,
        ),
        cstr.ParabolicPath.make(
            foot_r,
            init_pos=robot.pos,
            delta_x=x_dist,
            resolution=resolution,
        ),
    ))
    yield from robot.repeat_step(step_2, times=resolution)
    step_3 = cstr.CompoundConstraint((
        cstr.lock(face),
        cstr.Orbit.align(
            base-face,
            cstr.X + math.sqrt(3.)*cstr.Z,
            init_pos=robot.pos,
            resolution=resolution,
        ),
        cstr.xyz(chassis_com - face, y=0.),
        cstr.xyz(base - face, y=0.),
        cstr.lock(foot_l),
        cstr.lock(foot_r),
        cstr.Orbit.align(arm_l-foot_l, cstr.X, init_pos=robot.pos, resolution=resolution),
        cstr.Orbit.align(arm_r-foot_r, cstr.X, init_pos=robot.pos, resolution=resolution),
    ))
    yield from robot.repeat_step(step_3, times=resolution)
