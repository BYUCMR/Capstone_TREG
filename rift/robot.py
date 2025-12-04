from collections.abc import Generator, Iterable
from dataclasses import dataclass, field
from functools import partial
from typing import Self

import numpy as np

from . import steps
from .state import RobotState
from .truss_config import Lock, TrussConfig
from .tubetruss import TubeTruss
from .typing import Matrix, MatrixStack, Vector


class InverseKinematicsError(Exception): ...
class SolverError(InverseKinematicsError): ...
class SingularityError(InverseKinematicsError): ...


def initial_state(config: TrussConfig) -> RobotState:
    structure = config.triangles + config.payload
    n_rollers = sum(len(tube.rollers) for tube in structure)
    return RobotState(
        pos=config.initial_pos.copy(),
        roll=np.zeros(n_rollers),
    )


def near_singularity(H: Matrix, A: Matrix, c: float = 1e4) -> bool:
    m = len(A)
    O = np.zeros((m, m))
    K = np.block([[H, A.T], [A, O]])
    return np.linalg.cond(K) >= c


@dataclass(slots=True)
class RobotForward:
    structure: TubeTruss
    state: RobotState

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        return cls(config.triangles + config.payload, initial_state(config))

    @property
    def pos(self) -> Matrix:
        return self.state.pos

    @property
    def roll(self) -> Vector:
        return self.state.roll

    def update_state(self, roll: Vector, *, locks: Iterable[Lock] = ()) -> None:
        can_move = np.ones_like(self.pos, dtype=np.bool)
        for lock in locks:
            can_move[lock] = False
        unlocked_indices = np.flatnonzero(can_move)

        R = self.structure.norm_rigidity_at(self.pos)
        R_reduced = R[:, unlocked_indices]
        R_inv = np.linalg.inv(R_reduced)
        d_pos_reduced = R_inv @ self.structure.incidence @ (roll - self.roll)

        d_pos = np.zeros_like(self.pos)
        d_pos.put(unlocked_indices, d_pos_reduced)

        self.state = RobotState(roll=roll, pos=self.pos + d_pos)


@dataclass(slots=True)
class RobotInverse:
    structure: TubeTruss
    state: RobotState
    extra_constraints: Matrix | None = field(default=None, kw_only=True)

    @classmethod
    def from_config(cls, config: TrussConfig) -> Self:
        structure = config.triangles + config.payload
        state = initial_state(config)
        if config.keep_level is None:
            A_level = None
        else:
            A_level = np.zeros((1, state.pos.size))
            A_level[0, 3*config.keep_level[0]+2] =  1
            A_level[0, 3*config.keep_level[1]+2] = -1
        return cls(structure, state, extra_constraints=A_level)

    @property
    def pos(self) -> Matrix:
        return self.state.pos

    @property
    def roll(self) -> Vector:
        return self.state.roll

    def take_substep(self, substep: Matrix) -> None:
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
        dr = self.structure.incidence_inv @ rigidity @ dx.ravel()
        self.state = RobotState(
            pos=self.pos + dx,
            roll=self.roll + dr,
        )

    def take_step(self, step: MatrixStack) -> Generator[None]:
        for substep in step:
            self.take_substep(substep)
            yield

    def crawl(self, step_length: float = 0.8, *, resolution: int = 50) -> Generator[None]:
        feet = (0, 7, 6, 1)
        for foot in feet:
            locks = [(other_foot, 0.) for other_foot in feet if foot != other_foot]
            arc = partial(steps.parabola, d=step_length)
            step = steps.make_step_array(
                self.pos.shape, (foot, arc), *locks, resolution=resolution,
            )
            yield from self.take_step(step)
