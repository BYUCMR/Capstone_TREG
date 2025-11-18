from collections.abc import Generator, Iterable
from dataclasses import dataclass
from typing import Protocol, Self

import numpy as np
import qpsolvers

from . import steps
from .linalg import Matrix, Vector
from .state import RobotState
from .truss_config import Lock, TrussConfig
from .tubetruss import TubeTruss


def initial_state(config: TrussConfig) -> RobotState:
    structure = config.triangles + config.payload
    n_rollers = sum(len(tube.rollers) for tube in structure)
    return RobotState(
        pos=config.initial_pos.copy(),
        roll=np.zeros(n_rollers),
    )


def make_move_contraint(motion: Matrix) -> tuple[Matrix, Vector]:
    i = ~np.isnan(motion)
    b = motion[i]
    A = np.zeros((b.size, i.size))
    A[:, i.flat] = np.eye(len(A))
    return A, b


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

        R = self.structure.norm_rigidity_at(self.state)
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
        self.rigidity = self.structure.norm_rigidity_at(self.state)

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
        self.rigidity = self.structure.norm_rigidity_at(self.state)

    def make_constraint_matrices(self, motion: Matrix) -> tuple[Matrix, Vector]:
        A_move, b_move = make_move_contraint(motion)
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

    def get_optimal_motion(self, motion: Matrix) -> Vector:
        H = self.rigidity.T @ self.rigidity
        f = np.zeros(self.pos.size)
        A, b = self.make_constraint_matrices(motion)
        v = qpsolvers.solve_qp(P=H, q=f, A=A, b=b, solver='piqp')
        assert v is not None
        return v

    def take_step(self, step: steps.Step) -> Generator[tuple[Vector, Vector]]:
        motion = np.full_like(self.pos, np.nan)
        for lock in step.locks:
            motion[lock] = 0.
        while np.linalg.norm(error := step.target - self.pos[step.node]) > step.tol:
            node_vel = step.ctrl_func(error)
            yield self.pos[step.node], node_vel
            motion[step.node] = node_vel
            vel = self.get_optimal_motion(motion)
            self.update_state(vel * step.dt)

    def crawl(self, angle: float = 0, *, dt: float = 0.01) -> Generator[Matrix]:
        step_up = 0.4*np.array([np.cos(angle), np.sin(angle), 1.])
        step_down = 0.4*np.array([np.cos(angle), np.sin(angle), -1.])
        step_forward = step_up + step_down
        feet = (0, 7, 6, 1)
        for foot in feet:
            locks = [(other_foot, slice(0,3)) for other_foot in feet if foot != other_foot]
            path = np.vstack([
                self.pos[foot],
                self.pos[foot]+step_up,
                self.pos[foot]+step_forward,
            ])
            for point in path:
                step = steps.Step(node=foot, target=point, locks=locks, dt=dt)
                for _ in self.take_step(step):
                    yield path
