from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Protocol, Self, SupportsIndex

import numpy as np

from rift.arraytypes import Matrix, Vector
from rift.motion import constraints as cstr, steps
from rift.protocols import HasRigidity, StateFunction
from .linalg import cokernel, get_rigidity, incidence_from_trails

type RealMatrix = Matrix[np.integer | np.floating]


@dataclass(slots=True, frozen=True)
class ReachabilityConstraint(cstr.Constraint[HasRigidity]):
    """A constraint that ensures motion is possible for a robot."""
    unreachable: RealMatrix

    def at(self, state: HasRigidity) -> Matrix:
        A = self.unreachable @ state.rigidity
        b = np.zeros(len(self.unreachable))
        return np.column_stack((A, b))


@dataclass(slots=True, frozen=True)
class Actuation:
    """A description of the structure of a tube-truss robot."""
    forward: RealMatrix
    inverse: RealMatrix
    unreachable: RealMatrix

    def __post_init__(self) -> None:
        """Make sure this was constructed correctly."""
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


class HasActuation(Protocol):
    @property
    def actuation(self, /) -> Actuation: ...


@dataclass(slots=True)
class ActuatedTruss:
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
            raise ValueError("Truss position and incidence have mismatched node counts")
        if self._incidence.shape[0] != self.actuation.n_outputs:
            raise ValueError("Truss incidence and actuation have mismatched link counts")

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

    def nudge(self, dx: Matrix | Vector) -> Vector:
        """Modify the positions of the nodes of this truss."""
        if len(dx.shape) == 1:
            dx = dx.reshape(self._pos.shape)
        dq = self.actuation.inverse @ self.rigidity @ dx.ravel()
        self._rigidity = None
        self._pos[:] += dx
        return dq


class HasRigidityAndActuation(HasRigidity, HasActuation, Protocol): ...


class ControllableTruss(HasRigidityAndActuation, Protocol):
    def nudge(self, dx: Matrix | Vector, /) -> Vector: ...


class MotorCost:
    """
    A cost that scales with total motor motion.

    This class does not need to be instantiated.
    """

    @staticmethod
    def at(state: HasRigidityAndActuation) -> Matrix:
        return state.actuation.inverse @ state.rigidity


@dataclass(slots=True, frozen=True, kw_only=True)
class TrussController[T: ControllableTruss = ControllableTruss](steps.CanStep[T]):
    truss: T
    quad_cost: StateFunction[T, Matrix] = MotorCost
    lin_cost: StateFunction[T, Vector] | None = None

    @property
    def state(self) -> T:
        return self.truss

    def build_step(self, outline: steps.AbstractOutline[T]) -> steps.QPStep[T]:
        """Convert a step `Outline` into a `QPStep` suitable for use with this robot."""
        length_constraint = ReachabilityConstraint(self.truss.actuation.unreachable)
        outline = steps.combine_outlines(outline, steps.Outline(length_constraint))
        return steps.QPStep(self.quad_cost, self.lin_cost, outline)

    def nudge(self, dx: Matrix | Vector) -> Vector:
        """Modify the positions of the nodes of this robot."""
        return self.truss.nudge(dx)

    divide_steps = steps.divide_steps
