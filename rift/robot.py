from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from functools import partial
from typing import Self

import numpy as np

from . import steps
from .truss_config import Lock, TrussConfig
from .tubetruss import TubeTruss
from .typing import Matrix, Vector


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


def near_singularity(H: Matrix, A: Matrix, c: float = 1e4) -> bool:
    m = len(A)
    O = np.zeros((m, m))
    K = np.block([[H, A.T], [A, O]])
    return np.linalg.cond(K) >= c


@dataclass(slots=True)
class RobotForward:
    structure: TubeTruss
    pos: Matrix

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        return cls(config.triangles + config.payload, config.initial_pos.copy())

    def update_state(
        self,
        d_roll: Vector,
        *,
        locks: Iterable[Lock] = (),
    ) -> Matrix:
        can_move = np.ones_like(self.pos, dtype=np.bool)
        for lock in locks:
            can_move[lock] = False
        unlocked_indices = np.flatnonzero(can_move)

        R = self.structure.norm_rigidity_at(self.pos)
        R_reduced = R[:, unlocked_indices]
        R_inv = np.linalg.inv(R_reduced)
        d_pos_reduced = R_inv @ self.structure.incidence @ d_roll

        d_pos = np.zeros_like(self.pos)
        d_pos.put(unlocked_indices, d_pos_reduced)
        self.pos += d_pos
        return d_pos


@dataclass(slots=True)
class RobotInverse:
    structure: TubeTruss
    pos: Matrix
    extra_constraints: Matrix | None = field(default=None, kw_only=True)

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        structure = config.triangles + config.payload
        pos = config.initial_pos.copy()
        if config.keep_level is None:
            A_level = None
        else:
            A_level = np.zeros((1, pos.size))
            A_level[0, 3*config.keep_level[0]+2] =  1
            A_level[0, 3*config.keep_level[1]+2] = -1
        return cls(structure, pos, extra_constraints=A_level)

    def calculate_center_of_mass(
        self,
        *,
        joint_mass: float = 1.,
        payload_mass: float = 35.
    ) -> Vector:
        point_mass = np.zeros(12)
        point_mass[[0, 1, 5, 6, 7, 11]] = joint_mass
        point_mass[[2, 3, 4, 8, 9, 10]] = payload_mass / 6.
        total_mass = payload_mass + 6.*joint_mass
        relative_pos = self.pos - self.pos[0]
        return point_mass @ relative_pos / total_mass

    def take_substep(self, substep: Matrix) -> tuple[Matrix, Vector]:
        rigidity = self.structure.norm_rigidity_at(self.pos)
        A = self.structure.length_constraint @ rigidity
        if self.extra_constraints is not None:
            A = np.vstack([A, self.extra_constraints])
        dx = steps.fill_substep(substep, R=rigidity, A=A)
        if dx is None:
            raise SolverError("Could not find valid node velocities")
        # Instead of determining whether the configuration is approaching
        # a singularity, we just check to see if our velocity is going
        # out of control. It's computationally much faster.
        m1 = np.nanmax(substep)
        m2 = np.max(dx)
        if m2 >= 10.*m1:
            raise SingularityError("Robot configuration appears to be singular")
        self.pos += dx
        dr = self.structure.incidence_inv @ rigidity @ dx.ravel()
        return dx, dr

    def take_step(self, step: Iterable[Matrix]) -> Generator[tuple[Matrix, Vector]]:
        for substep in step:
            yield self.take_substep(substep)

    def crawl(
        self,
        cycles: int = 1,
        step_length: float = 0.8,
        *,
        resolution: int = 50,
    ) -> Generator[tuple[Matrix, Vector]]:
        feet = (1, 0, 7, 6)
        for foot in (feet * cycles):
            locks = [(other_foot, 0.) for other_foot in feet if foot != other_foot]
            arc = partial(steps.parabola, d=step_length)
            line = partial(steps.line, d=step_length/4)
            step = steps.make_step_array(
                self.pos.shape,
                (foot, arc),
                (2, line),
                (8, line),
                *locks,
                resolution=resolution,
            )
            yield from self.take_step(step)
