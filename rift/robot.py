from collections.abc import Generator
from dataclasses import dataclass

import numpy as np

from . import constrain as cstr
from . import steps
from .arraytypes import Matrix, Vector
from .tubetruss import Truss, LengthControl


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


@dataclass(slots=True)
class TrussRobot:
    """
    A representation of a truss robot.

    It comprises a position, a truss structure, and a control setup.
    """
    pos: Matrix
    truss: Truss
    control: LengthControl

    def apply_roll(
        self,
        d_roll: Vector,
        *constraints: cstr.Constraint,
        t: float = 0.,
    ) -> Matrix:
        rigidity = self.truss.rigidity_at(self.pos)
        constraint = cstr.CompoundConstraint((
            cstr.CustomConstraint(self.control.unreachable @ rigidity),
            *constraints
        ))
        A, b = constraint.get(self.pos, t)
        d_length = self.control.forward @ d_roll
        d_pos = np.linalg.solve(
            np.concat((rigidity, A)),
            np.concat((d_length, b)),
        )
        self.pos += d_pos
        return d_pos

    def take_substep(
        self,
        *constraints: cstr.Constraint,
        t: float = 0.,
        allow_redundant: bool = False,
    ) -> tuple[Matrix, Vector]:
        rigidity = self.truss.rigidity_at(self.pos)
        constraint = cstr.CompoundConstraint((
            cstr.CustomConstraint(self.control.unreachable @ rigidity),
            *constraints
        ))
        A, b = constraint.get(self.pos, t)
        e, v = cstr.singularity_eig(A, b if allow_redundant else None)
        if abs(e) <= 1e-3:
            raise SingularityError("Robot state is singular")
        dx = steps.find_dx(R=rigidity, A=A, b=b, solver='piqp' if allow_redundant else 'kkt')
        if dx is None:
            raise SolverError("Could not find valid node velocities")
        dx = dx.reshape(self.pos.shape)
        self.pos += dx
        dr = self.control.inverse @ rigidity @ dx.ravel()
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
