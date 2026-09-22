from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import ClassVar, Protocol, Self, SupportsIndex

import numpy as np

from rift.arraytypes import Matrix, Vector
from rift.motion import steps
from rift.protocols import HasRigidity, StateFunction, SupportsNudge
from .linalg import cokernel, get_rigidity, incidence_from_trails

type RealMatrix = Matrix[np.integer | np.floating]


@dataclass(slots=True, frozen=True)
class Actuation:
    """A description of the structure of a tube-truss robot."""
    _forward: RealMatrix
    _inverse: RealMatrix
    _unreachable: RealMatrix

    def __post_init__(self) -> None:
        """Make sure this was constructed correctly."""
        if self._forward.shape != self._inverse.T.shape:
            raise ValueError("Forward and inverse matrices have mismatched shapes")
        if self._inverse.shape[1] != self._unreachable.shape[1]:
            raise ValueError("Inverse and unreachable matrices have unequal column counts")
        if np.any(self._unreachable @ self._forward):
            raise ValueError("Unreachable null space does not contain forward column space")
        round_trip = self._inverse @ self._forward
        identity = np.eye(self._forward.shape[1])
        if not np.allclose(round_trip, identity):
            raise ValueError("Inverse matrix times forward matrix is not an identity")

    @property
    def forward(self) -> RealMatrix:
        view = self._forward.view()
        view.setflags(write=False)
        return view

    @property
    def inverse(self) -> RealMatrix:
        view = self._inverse.view()
        view.setflags(write=False)
        return view

    @property
    def unreachable(self) -> RealMatrix:
        view = self._unreachable.view()
        view.setflags(write=False)
        return view

    @property
    def n_inputs(self) -> int:
        return len(self._inverse)

    @property
    def n_outputs(self) -> int:
        return len(self._forward)

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


class HasRigidityAndActuation(HasRigidity, HasActuation, Protocol): ...


@dataclass(slots=True, frozen=True)
class ReachabilityConstraint:
    """
    A constraint that ensures motion is possible for a robot.

    This class does not need to be instantiated.
    """

    @staticmethod
    def at(state: HasRigidityAndActuation) -> Matrix:
        A = state.actuation.unreachable @ state.rigidity
        b = np.zeros(len(state.actuation.unreachable))
        return np.column_stack((A, b))


@dataclass(slots=True)
class ActuatedTruss:
    """
    A representation of a truss robot.

    It comprises a position, a truss topology, and an actuation schema.
    """
    stipulation: ClassVar = steps.Outline(ReachabilityConstraint)
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
            self._rigidity.setflags(write=False)
        return self._rigidity

    def nudge(self, dx: Matrix | Vector) -> Vector:
        """Modify the positions of the nodes of this truss."""
        if len(dx.shape) == 1:
            dx = dx.reshape(self._pos.shape)
        dq = self.actuation.inverse @ self.rigidity @ dx.ravel()
        self._rigidity = None
        self._pos[:] += dx
        return dq


class ControllableTruss(HasRigidityAndActuation, steps.SupportsNudge, Protocol): ...


class MotorCost:
    """
    A cost that scales with total motor motion.

    This class does not need to be instantiated.
    """

    @staticmethod
    def at(state: HasRigidityAndActuation) -> Matrix:
        return state.actuation.inverse @ state.rigidity


@dataclass(slots=True, frozen=True)
class TrussController[T: SupportsNudge](steps.CanStep[T]):
    truss: T
    solver: steps.Solver[T]

    @classmethod
    def make_default[CT: ControllableTruss](
        cls: type['TrussController[CT]'],
        truss: CT,
        *,
        quad_cost: StateFunction[CT, Matrix] = MotorCost,
        lin_cost: StateFunction[CT, Vector] | None = None,
        eig_threshold: float = 1e-3,
        allow_redundancy: bool = False,
    ) -> 'TrussController[CT]':
        return cls(
            truss, steps.QPSolver(
                quad_cost, lin_cost,
                eig_threshold=eig_threshold,
                allow_redundancy=allow_redundancy
            )
        )

    def solve(self, outline: steps.AbstractOutline[T]) -> Vector:
        return self.solver.solve(outline, self.truss)

    def nudge(self, dx: Vector) -> Vector:
        """Modify the positions of the nodes of this robot."""
        return self.truss.nudge(dx)

    divide_steps = steps.divide_steps
