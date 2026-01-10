from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from functools import partial
from typing import Self

import numpy as np

from . import steps
from .arraytypes import Matrix, Vector
from .truss_config import Lock, TrussConfig
from .tubetruss import TubeTruss


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


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

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        structure = config.triangles + config.payload
        pos = config.initial_pos.copy()
        return cls(structure, pos)

    def take_substep(
        self,
        substep: Matrix,
        constraints: Matrix | None = None,
    ) -> tuple[Matrix, Vector]:
        rigidity = self.structure.norm_rigidity_at(self.pos)
        A = self.structure.length_constraint @ rigidity
        if constraints is not None:
            A = np.vstack([A, constraints])
        dx, det = steps.fill_substep(substep, R=rigidity, A=A)
        if dx is None:
            raise SolverError("Could not find valid node velocities")
        if np.abs(det) < 0.05:
            raise SingularityError("Robot state is nearly singular")
        self.pos += dx
        dr = self.structure.incidence_inv @ rigidity @ dx.ravel()
        return dx, dr

    def take_step(
        self,
        step: Iterable[Matrix],
        constraints: Matrix | None = None,
    ) -> Generator[tuple[Matrix, Vector]]:
        for substep in step:
            yield self.take_substep(substep, constraints)

    def crawl(
        self,
        cycles: int = 1,
        step_length: float = 0.8,
        *,
        resolution: int = 50,
    ) -> Generator[tuple[Matrix, Vector]]:
        constraints = np.zeros((1, self.pos.size))
        constraints[0, 3*2+2] =  1.
        constraints[0, 3*8+2] = -1.
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
            yield from self.take_step(step, constraints=constraints)
