from collections.abc import Generator, Iterable
from dataclasses import dataclass
from functools import partial
from typing import Protocol, Self

import numpy as np
import qpsolvers

from . import steps
from .linalg import Matrix, MatrixStack, Vector
from .state import RobotState
from .truss_config import Lock, TrussConfig
from .tubetruss import TubeTruss


class SingularityError(ValueError):
    pass


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


class Robot(Protocol):
    @property
    def structure(self) -> TubeTruss: ...
    @property
    def state(self) -> RobotState: ...


@dataclass
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


class RobotInverse:
    def __init__(self, config: TrussConfig) -> None:
        self.keep_level = config.keep_level
        self.structure = config.triangles + config.payload
        self.state = initial_state(config)
        self.rigidity = self.structure.norm_rigidity_at(self.state.pos)

    @property
    def pos(self) -> Matrix:
        return self.state.pos

    @property
    def roll(self) -> Vector:
        return self.state.roll

    def update_state(self, d_pos: Vector) -> None:
        d_roll = self.structure.incidence_inv @ self.rigidity @ d_pos
        d_pos_mat = d_pos.reshape(self.pos.shape)
        self.state = RobotState(
            pos=self.pos + d_pos_mat,
            roll=self.roll + d_roll,
        )
        self.rigidity = self.structure.norm_rigidity_at(self.pos)

    def make_constraint_matrices(self, substep: Matrix) -> tuple[Matrix, Vector]:
        A_move, b_move = steps.make_move_constraint(substep)
        if self.keep_level is not None:
            A_level = np.zeros((1, self.pos.size))
            A_level[0, 3*self.keep_level[0]+2] =  1
            A_level[0, 3*self.keep_level[1]+2] = -1
            A_move = np.vstack([A_move, A_level])
            b_move = np.concat([b_move, [0.]])

        A_length = self.structure.length_constraint @ self.rigidity
        b_length = np.zeros((len(A_length),))

        Aeq = np.vstack([A_move, A_length])
        beq = np.concat([b_move, b_length])
        return Aeq, beq

    def get_optimal_motion(self, substep: Matrix) -> Vector:
        H = self.rigidity.T @ self.rigidity
        f = np.zeros(self.pos.size)
        A, b = self.make_constraint_matrices(substep)
        v = qpsolvers.solve_qp(P=H, q=f, A=A, b=b, solver='piqp')
        assert v is not None
        # Instead of determining whether the configuration is approaching
        # a singularity, we just check to see if our velocity is going
        # out of control. It's computationally much faster.
        m1 = np.nanmax(substep)
        m2 = np.max(v)
        if m2 >= 10.*m1:
            raise SingularityError
        return v

    def take_substep(self, substep: Matrix) -> None:
        d_pos = self.get_optimal_motion(substep)
        self.update_state(d_pos)

    def take_step(self, step: MatrixStack) -> Generator[None]:
        for substep in step:
            self.take_substep(substep)
            yield

    def crawl(self, step_length: float = 0.8, *, resolution: int = 50) -> Generator[Matrix]:
        feet = (0, 7, 6, 1)
        for foot in feet:
            locks = [(other_foot, 0.) for other_foot in feet if foot != other_foot]
            arc = partial(steps.parabola, d=step_length)
            step = steps.make_step_array(
                self.pos.shape, (foot, arc), *locks, resolution=resolution,
            )
            path = self.pos[foot] + np.cumsum(step[:, foot, :], axis=0)
            for _ in self.take_step(step):
                yield path
