import math
from dataclasses import dataclass, field
from typing import Final, Literal, Self

import numpy as np

from .mathtools import check_inside_and_closest_edge
from .truss_config import TrussConfig
from .typing import Matrix, Vector

DEFAULT_TOL: Final = 1e-6


def get_contact_transform(
    old_pos: Matrix,
    new_pos: Matrix,
    *,
    tol: float = DEFAULT_TOL,
) -> Matrix | None:
    i = np.argmin(new_pos[:, 2])
    min_z = new_pos[i, 2]
    if min_z >= -tol:
        return None
    delta = new_pos[i] - old_pos[i]
    xform = np.eye(4)
    if math.isclose(delta[2], 0.):
        # It's underground due to tipping.
        # TODO: Should we handle this better?
        xform[2, 3] = -min_z
    else:
        xform[:3, 3] = -min_z / delta[2] * delta
    return xform


def get_fall_transform(
    pos: Matrix,
    com: Vector,
    *,
    gravity: Vector = np.array([0., 0., -1.]),
    tol: float = DEFAULT_TOL,
) -> Matrix | None:
    contacts = pos[np.abs(pos[:, 2]) <= tol]
    if len(contacts) == 0:
        xform = np.eye(4)
        xform[:3, 3] = np.min(pos[:, 2]) / gravity[2] * gravity
        return xform
    elif len(contacts) == 1:
        origin = contacts[0]
        direction = np.cross(com - origin, gravity)
    elif len(contacts) == 2:
        origin = contacts[0]
        direction = contacts[1] - origin
    else:
        com_on_ground = com[:2] - com[2] / gravity[2] * gravity[:2]
        _, _, edge, _ = check_inside_and_closest_edge(com_on_ground, contacts[:,:2])
        if edge is None:
            return None
        i, j = edge
        origin = contacts[i]
        direction = contacts[j] - origin
    if np.cross(direction, com - origin)[2] > 0:
        direction = -direction
    rel_pos = pos[pos[:, 2] > tol] - origin
    T1, T2 = np.eye(4), np.eye(4)
    T1[:3, 3] = -origin
    T2[:3, 3] =  origin
    T2[:3, :3] = tipping_rotation(rel_pos, direction)
    return T2 @ T1


def tipping_rotation(rel_pos: Matrix, rel_axis: Vector) -> Matrix:
    unit_axis = rel_axis / np.linalg.norm(rel_axis)
    rel_z = rel_pos[:, 2]
    rel_x = np.vecdot(rel_pos, np.cross(unit_axis, [0., 0., 1.]))
    i = np.argmin(np.atan2(rel_z, rel_x))  # TODO: Can we take a shortcut here?
    hypot = math.hypot(rel_z[i], rel_x[i])
    sine = rel_z[i] / hypot
    cosine = rel_x[i] / hypot
    # This is a form of Rodrigues' formula.
    I = np.eye(3)
    K = np.linalg.cross(I, unit_axis)
    return I + K * sine + K@K * (1. - cosine)


@dataclass
class Stabilizer:
    source_pos: Matrix
    xform: Matrix = field(default_factory=lambda: np.eye(4))
    gravity: Vector = field(default_factory=lambda: np.array([0., 0., -1.]), kw_only=True)
    tol: float = field(default=DEFAULT_TOL, kw_only=True)
    rel_mass: Vector | Literal[1] = field(default=1, kw_only=True)

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        source_pos = config.initial_pos.copy()
        rel_mass = config.mass / np.sum(config.mass)
        return cls(source_pos, rel_mass=rel_mass)

    def apply_xform(self, pos: Matrix) -> Matrix:
        hom_pos = np.hstack([pos, np.ones((len(pos), 1))])
        return np.matvec(self.xform, hom_pos)[:, :3]

    @property
    def pos(self) -> Matrix:
        return self.apply_xform(self.source_pos)

    def adjust_for(self, source_pos: Matrix) -> bool:
        pos = self.apply_xform(source_pos)
        push_xform = get_contact_transform(self.pos, pos, tol=self.tol)
        if push_xform is not None:
            self.xform = push_xform @ self.xform
            return True
        com = self.rel_mass @ pos
        fall_xform = get_fall_transform(pos, com, gravity=self.gravity, tol=self.tol)
        if fall_xform is not None:
            self.xform = fall_xform @ self.xform
            return True
        return False

    def update_pos(self, source_pos: Matrix) -> None:
        while self.adjust_for(source_pos):
            pass
        self.source_pos = source_pos
