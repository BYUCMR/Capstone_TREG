from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from typing import SupportsIndex

import numpy as np

from . import steps
from .arraytypes import Matrix, Vector
from .tubetruss import TubeTruss


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


type Index = SupportsIndex | slice[SupportsIndex]
type Lock = tuple[Index, Index]


@dataclass(slots=True)
class RobotForward:
    structure: TubeTruss
    pos: Matrix

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
        d_pos_reduced = R_inv @ self.structure.roll_to_length @ d_roll

        d_pos = np.zeros_like(self.pos)
        d_pos.put(unlocked_indices, d_pos_reduced)
        self.pos += d_pos
        return d_pos


@dataclass(slots=True)
class RobotInverse:
    structure: TubeTruss
    pos: Matrix

    def take_substep(
        self,
        substep: Matrix,
        constraints: Matrix | None = None,
    ) -> tuple[Matrix, Vector]:
        rigidity = self.structure.norm_rigidity_at(self.pos)
        A = self.structure.length_summer @ rigidity
        if constraints is not None:
            A = np.vstack([A, constraints])
        dx, det = steps.fill_substep(substep, R=rigidity, A=A)
        if dx is None:
            raise SolverError("Could not find valid node velocities")
        if np.abs(det) < 0.05:
            raise SingularityError("Robot state is nearly singular")
        self.pos += dx
        dr = self.structure.length_to_roll @ rigidity @ dx.ravel()
        return dx, dr

    def take_step(
        self,
        step: Iterable[Matrix],
        constraints: Matrix | None = None,
    ) -> Generator[tuple[Matrix, Vector]]:
        for substep in step:
            yield self.take_substep(substep, constraints)
