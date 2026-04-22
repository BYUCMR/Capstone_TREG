from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Self, SupportsIndex

import numpy as np

from rift.arraytypes import Matrix, Vector
from rift.motion import constraints as cstr, steps
from .linalg import cokernel, get_rigidity, incidence_from_trails

type RealMatrix = Matrix[np.integer | np.floating]


@dataclass(slots=True, frozen=True)
class Actuation:
    forward: RealMatrix
    inverse: RealMatrix
    unreachable: RealMatrix

    def __post_init__(self) -> None:
        if self.forward.shape != self.inverse.T.shape:
            raise ValueError("Forward and inverse matrices have mismatched shapes")
        if self.inverse.shape[1] != self.unreachable.shape[1]:
            raise ValueError("Inverse and unreachable matrices have unequal column counts")
        if np.any(self.unreachable @ self.forward):
            raise ValueError("Unreachable null space does not contain forward column space")
        round_trip = self.inverse @ self.forward
        identity = np.eye(self.forward.shape[1])
        if not np.allclose(round_trip, identity):
            raise ValueError("Inverse matrix times forward matrix is not an identity")

    @property
    def n_inputs(self) -> int:
        return len(self.inverse)

    @property
    def n_outputs(self) -> int:
        return len(self.forward)

    @classmethod
    def make_simple(cls, n: int) -> Self:
        return cls(np.eye(n), np.eye(n), np.zeros((0, n)))

    @classmethod
    def from_unreachable(cls, unreachable: RealMatrix) -> Self:
        forward = cokernel(unreachable.T).T
        inverse = np.linalg.pinv(forward)
        return cls(forward, inverse, unreachable)

    @classmethod
    def from_forward(cls, forward: RealMatrix) -> Self:
        inverse = np.linalg.pinv(forward)
        unreachable = cokernel(forward)
        return cls(forward, inverse, unreachable)

    @classmethod
    def from_trails(
        cls,
        *trails: Iterable[SupportsIndex],
        n_static: int = 0,
    ) -> Self:
        forward_T = incidence_from_trails(*trails, empty_cols=n_static)
        return cls.from_forward(forward_T.T)


@dataclass(slots=True)
class TrussRobot(steps.MovableRobot):
    """
    A representation of a truss robot.

    It comprises a position, a truss topology, and an actuation schema.
    """
    _pos: Matrix
    _incidence: Matrix[np.int8]
    actuation: Actuation
    # We cache rigidity because it's a little slow to compute.
    _rigidity: Matrix | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self._pos.shape[0] != self._incidence.shape[1]:
            raise ValueError("Robot position and incidence have mismatched node counts")
        if self._incidence.shape[0] != self.actuation.n_outputs:
            raise ValueError("Robot incidence and actuation have mismatched link counts")

    @property
    def pos(self) -> Matrix:
        view = self._pos.view()
        view.setflags(write=False)
        return view

    @property
    def incidence(self) -> Matrix[np.int8]:
        view = self._incidence.view()
        view.setflags(write=False)
        return view

    @property
    def rigidity(self) -> Matrix:
        if self._rigidity is None:
            self._rigidity = get_rigidity(self._incidence, self._pos)
        return self._rigidity

    @property
    def dx_to_dq(self) -> Matrix:
        return self.actuation.inverse @ self.rigidity

    def resolve_constraint(self, constraint: cstr.Constraint) -> Matrix:
        length_constraint = cstr.Static.make_hom(self.actuation.unreachable @ self.rigidity)
        constraint = cstr.combine(length_constraint, constraint)
        return constraint.at(self.pos)

    def nudge(self, dx: Matrix | Vector) -> None:
        if len(dx.shape) == 1:
            dx = dx.reshape(self._pos.shape)
        self._rigidity = None
        self._pos[:] += dx

    take_step = steps.take_step
    divide_steps = steps.divide_steps
