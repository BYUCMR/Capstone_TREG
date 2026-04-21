import enum
import math
from collections.abc import Callable, Generator
from dataclasses import dataclass
from functools import partial
from typing import Final, Self, SupportsIndex

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl

from . import anim
from . import grav
from . import tubetruss as tt
from .arraytypes import Matrix, Vector
from .motion import axes, constraints as cstr, points, steps


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

    def __sub__(self, other: points.Point) -> Vector:
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


# Truss structure
INCIDENCE: Final = tt.incidence_from_trails(
    (L1, L2, P3, L1),
    (P1, L2, L3, P1),
    (L1, P2, L3, L1),
    (R1, R2, Q3, R1),
    (Q1, R2, R3, Q1),
    (R1, Q2, R3, R1),
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
INCIDENCE.setflags(write=False)

# Roller setup
ACTUATION: Final = tt.Actuation.from_trails(
    # These indices are based on the construction order above.
    ( 2,  0,  1),  # Rollers 01 and 02; P3-L1-L2-P3
    ( 3,  4,  5),  # Rollers 03 and 04; P1-L2-L3-P1
    ( 7,  8,  6),  # Rollers 05 and 06; P2-L3-L1-P2
    (10,  9, 11),  # Rollers 07 and 08; Q3-R2-R1-Q3
    (15, 17, 16),  # Rollers 09 and 10; Q2-R1-R3-Q2
    (14, 13, 12),  # Rollers 11 and 12; Q1-R3-R2-Q1
    n_static=12,   # The number of chassis links
)


# Centers of mass
COM: Final = np.zeros(12)
CHASSIS_COM: Final = np.zeros(12)
COM[:6] = 2.
COM[6:] = CHASSIS_COM[6:] = 1.
COM[:] /= np.sum(COM)
CHASSIS_COM[:] /= np.sum(CHASSIS_COM)
COM.setflags(write=False)
CHASSIS_COM.setflags(write=False)


# Physical size
# In actuality, we have 1125 * 12 * 4 ticks per side, but we
# increase this to account for not simulating node offsets.
TICKS_PER_SIDE: Final = 1125 * 12 * 4 * 2


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


def make_robot(init_pos: Matrix = CRAWLING_POS) -> 'Rover':
    return Rover.make_pos(init_pos)


def make_stabilizer(init_pos: Matrix = CRAWLING_POS) -> grav.Stabilizer:
    source_pos = init_pos.copy()
    return grav.Stabilizer(source_pos, rel_mass=COM)


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
    chassis_links = np.array([np.flatnonzero(row) for row in INCIDENCE[18:]])
    items.append(anim.draw_links(
        chassis_links.ravel(),
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


_identity = np.eye(3, dtype=np.bool)
ROLL: Final = np.kron(np.eye(4, dtype=np.bool), np.kron(_identity[[1, 2, 0]], _identity))
ROLL.setflags(write=False)
del _identity


@dataclass(slots=True)
class Rover(steps.MovableRobot):
    source: tt.TrussRobot
    permuter: Matrix[np.bool]

    @classmethod
    def make_pos(cls, pos: Matrix) -> Self:
        return cls(tt.TrussRobot(pos.copy(), INCIDENCE, ACTUATION), np.eye(pos.size, dtype=np.bool))

    @property
    def pos(self) -> Matrix:
        return (self.permuter.T @ self.source.pos.ravel()).reshape(self.source.pos.shape)

    @property
    def incidence(self) -> Matrix[np.int8]:
        return self.source.incidence

    @property
    def dx_to_dq(self) -> Matrix:
        return self.source.dx_to_dq @ self.permuter

    def nudge(self, dx: Vector) -> None:
        self.source.nudge(self.permuter @ dx)

    def resolve_constraint(self, constraint: cstr.Constraint) -> Matrix:
        permuted = cstr.Permuted(constraint, self.permuter.T)
        aug = self.source.resolve_constraint(permuted)
        aug[:, :-1] @= self.permuter
        return aug

    take_step = steps.take_step
    divide_steps = steps.divide_steps


def roller_adjustment(roller: SupportsIndex, amount: float) -> steps.Step[Rover]:
    dq = np.zeros(len(Node))
    dq[roller] = amount
    # We might be able to make better constraints than this.
    constraint = cstr.Static.combine(
        cstr.xyz(L1, x=0, y=0, z=0),
        cstr.xyz(L2, y=0, z=0),
        cstr.xyz(R1, z=0),
    )
    return steps.FKStep(dq, constraint)


def node_nudge(node: SupportsIndex, x: float, y: float, z: float) -> steps.Step[Rover]:
    def move(pos: Matrix) -> cstr.Static:
        locked = pos[:, 2] < 1e-6
        locked[node] = False
        return cstr.Static.combine(
            cstr.xyz(Node(node), x, y, z),
            *(
                cstr.lock(Node(n))
                for n in np.flatnonzero(locked)
            ),
        )
    def respect_floor(pos: Matrix) -> cstr.PlanarBarrier:
        return cstr.PlanarBarrier(np.eye(len(pos)), axes.Z)
    return steps.QPStep(
        cstr.Sleeper(move),
        cstr.Sleeper(respect_floor),
        allow_redundancy=True,
    )


def chassis_nudge(x: float, y: float, z: float) -> steps.Step[Rover]:
    return steps.KKTStep(cstr.Static.combine(
        cstr.xyz(CHASSIS_COM, x, y, z),
        cstr.xyz(P3-Q3, x=0, z=0),
        cstr.xyz(L1, z=0),
        cstr.xyz(L2, z=0),
        cstr.xyz(R1, z=0),
        cstr.xyz(R2, z=0),
        cstr.xyz(points.centroid(L1, L2, R1, R2), x=0, y=0),
    ))


def chassis_tilt(angle: float) -> steps.Step[Rover]:
    base = points.centroid(P3, Q3)
    face = points.centroid(P2, Q2)
    return steps.KKTStep(cstr.Compound((
        cstr.lock(base),
        cstr.Orbit(face-base, axes.Y, angle),
        cstr.xyz(face - base, y=0.),
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.xyz(L2, y=0.),
        cstr.xyz(R2, y=0.),
        cstr.xyz(L3, y=0.),
        cstr.xyz(R3, y=0.),
    )))


def crawl(
    cycles: int = 1,
    step_length: tuple[float, float] = (0.125, 0.),
) -> Generator[steps.Step[Rover]]:
    chassis_up = CHASSIS_COM - points.centroid(P3, Q3)
    no_wobble = cstr.motion(chassis_up, np.eye(3)[0:2], np.zeros(2))
    x_dist, y_dist = step_length
    steadily_forward = cstr.xyz(CHASSIS_COM, x=0.25 * x_dist)
    feet = (L2, L1, R2, R1)
    for foot in (feet * cycles):
        yield steps.KKTStep(cstr.Compound((
            cstr.ParabolicPath.make(
                point=foot,
                delta_x=x_dist,
                delta_y=y_dist,
            ),
            *(
                cstr.lock(other_foot)
                for other_foot in feet
                if foot is not other_foot
            ),
            steadily_forward,
            no_wobble,
        )))


def shuffle(x_dist: float = 0.125) -> Generator[steps.Step[Rover]]:
    chassis_up = CHASSIS_COM - points.centroid(P3, Q3)
    no_wobble = cstr.motion(chassis_up, np.eye(3)[0:2], np.zeros(2))
    c_dist = x_dist * 0.4
    yield steps.KKTStep(cstr.Compound((
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.lock(L2),
        cstr.lock(R2),
        cstr.xyz(COM, x=c_dist),
        no_wobble,
    )))
    yield steps.KKTStep(cstr.Compound((
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.Static.xyz(L2, x_dist, 0., 0.),
        cstr.Static.xyz(R2, x_dist, 0., 0.),
        cstr.xyz(COM, x=0.5*x_dist-c_dist),
        no_wobble,
    )))
    yield steps.KKTStep(cstr.Compound((
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.lock(L2),
        cstr.lock(R2),
        cstr.xyz(COM, x=-c_dist),
        no_wobble,
    )))
    yield steps.KKTStep(cstr.Compound((
        cstr.Static.xyz(L1, x_dist, 0., 0.),
        cstr.Static.xyz(R1, x_dist, 0., 0.),
        cstr.lock(L2),
        cstr.lock(R2),
        cstr.xyz(COM, x=0.5*x_dist+c_dist),
        no_wobble,
    )))


def lean(dist: float = 0.6) -> steps.Step[Rover]:
    return steps.KKTStep(cstr.Static.combine(
        cstr.xyz(P2, dist),
        cstr.xyz(Q2, dist),
        cstr.lock(L1),
        cstr.lock(R1),
        cstr.lock(L2),
        cstr.lock(R2),
    ))


def reach(dist: float = 1.) -> steps.Step[Rover]:
    return steps.KKTStep(cstr.Static.combine(
        cstr.xyz(L3, x=dist),
        cstr.xyz(R3, x=dist),
        cstr.lock(P3),
        cstr.lock(Q3),
        cstr.lock(L1),
        cstr.lock(R1),
    ))


def roll() -> Generator[steps.Step[Rover]]:
    base = points.centroid(P3, Q3)
    face = points.centroid(P2, Q2)
    back = points.centroid(P1, Q1)
    feet_midpoint = points.centroid(L1, R1)
    yield steps.KKTStep(cstr.Compound((
        cstr.lock(base),
        cstr.xyz(COM - feet_midpoint, x=0.25),
        cstr.Orbit.align(face - base, axes.X),
        cstr.xyz(P3 - Q3, x=0., z=0.),
        cstr.xyz(L1, z=0.),
        cstr.xyz(R1, z=0.),
    )))
    def step_back(x: Matrix) -> cstr.Constraint:
        x0 = (face @ x)[0] - 0.5
        o_l = np.array((x0, x[L1, 1], 0.))
        o_r = np.array((x0, x[R1, 1], 0.))
        return cstr.Compound((
            cstr.ParabolicPath(L1, o_l, abs(x[L1, 0] - o_l), 0.5),
            cstr.ParabolicPath(R1, o_r, abs(x[R1, 0] - o_l), 0.5),
        ))
    yield steps.KKTStep(cstr.Compound((
        cstr.lock(CHASSIS_COM),
        cstr.xyz(P2 - Q2, z=0.),
        cstr.xyz(face - base, y=0., z=0.),
        cstr.Sleeper(step_back),
        cstr.Radial.get_to(L3-L1, 1.),
        cstr.Radial.get_to(R3-R1, 1.),
        cstr.Orbit.align(L3-L1, axes.X, axis=axes.Y),
        cstr.Orbit.align(R3-R1, axes.X, axis=axes.Y),
    )))
    def align_feet(x: Matrix) -> cstr.Constraint:
        point_l = L3 - L1
        point_r = R3 - R1
        target = (1., 0., 0.)
        return cstr.Compound((
            cstr.Static.motion(point_l, np.eye(3), target - (point_l @ x)),
            cstr.Static.motion(point_r, np.eye(3), target - (point_r @ x)),
        ))
    yield steps.KKTStep(cstr.Compound((
        cstr.lock(face),
        cstr.Orbit.align(back - base, axes.X),
        cstr.xyz(P2 - Q2, x=0., z=0.),
        cstr.xyz(L1, x=0., z=0.),
        cstr.xyz(R1, x=0., z=0.),
        cstr.Sleeper(align_feet),
    )))
