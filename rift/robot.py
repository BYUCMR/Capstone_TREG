from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from typing import SupportsIndex

import numpy as np

from . import constrain as cstr
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
    roll_to_length: Matrix
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
        d_pos_reduced = R_inv @ self.roll_to_length @ d_roll

        d_pos = np.zeros_like(self.pos)
        d_pos.put(unlocked_indices, d_pos_reduced)
        self.pos += d_pos
        return d_pos


@dataclass(slots=True)
class RobotInverse:
    structure: TubeTruss
    length_to_roll: Matrix
    pos: Matrix

    @property
    def length_constraint(self) -> cstr.FixedLength:
        return cstr.FixedLength(
            self.structure.pos_to_rigidity,
            self.structure.length_summer,
        )

    def take_substep(
        self,
        *constraints: cstr.Constraint,
        t: float = 0.,
        allow_redundant: bool = False,
    ) -> tuple[Matrix, Vector]:
        rigidity = self.structure.norm_rigidity_at(self.pos)
        constraint = cstr.CompoundConstraint((
            self.length_constraint, *constraints
        ))
        A, b = constraint.get(self.pos, t)
        e, v = cstr.singularity_eig(A, b if allow_redundant else None)
        if abs(e) <= 1e-3:
            raise SingularityError("Robot state is singular")
        dx = steps.find_dx(R=rigidity, A=A, b=b)
        if dx is None:
            raise SolverError("Could not find valid node velocities")
        dx = dx.reshape(self.pos.shape)
        self.pos += dx
        dr = self.length_to_roll @ rigidity @ dx.ravel()
        return dx, dr

    def take_step(
        self,
        *constraints: cstr.Constraint,
        resolution: int,
        allow_redundant: bool = False,
    ) -> Generator[tuple[Matrix, Vector]]:
        for t in np.linspace(0., 1., resolution):
            yield self.take_substep(
                *constraints,
                t=t,
                allow_redundant=allow_redundant,
            )
