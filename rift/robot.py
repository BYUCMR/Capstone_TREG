from collections.abc import Generator, Iterable
from dataclasses import dataclass
from functools import partial
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


def step_arc(t: Vector, d: float = 1.0) -> Matrix:
    k = d / len(t)
    u = np.full_like(t, k)
    v = np.zeros_like(t)
    w = 2. * k * (0.5-t)
    return np.array([u, v, w])


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

    @property
    def center_of_mass(self):
        joint_mass = 1
        payload_mass = 35
        point_mass = np.array([joint_mass,joint_mass,1/6*payload_mass,1/6*payload_mass,1/6*payload_mass,joint_mass,joint_mass,joint_mass,1/6*payload_mass,1/6*payload_mass,1/6*payload_mass,joint_mass])
        dist_x = []
        dist_y = []
        dist_z = []
        for x,y,z in self.pos:
            dist_x.append(x - self.pos[0][0])
            dist_y.append(y - self.pos[0][1])
            dist_z.append(z - self.pos[0][2])
        x_com = np.sum(np.multiply(point_mass, np.array(dist_x)))/(6*joint_mass+payload_mass)
        y_com = np.sum(np.multiply(point_mass, np.array(dist_y)))/(6*joint_mass+payload_mass)
        z_com = np.sum(np.multiply(point_mass, np.array(dist_z)))/(6*joint_mass+payload_mass)
        return x_com,y_com,z_com

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

    def take_substep(self, motion: Matrix) -> None:
        d_pos = self.get_optimal_motion(motion)
        self.update_state(d_pos)

    def take_step(self, step: steps.Step, *, resolution: int = 10) -> Generator[Matrix]:
        motion = steps.make_motion_array(step, self.pos.shape, resolution=resolution)
        path = self.pos[step.node] + np.cumsum(motion[:, step.node, :], axis=0)
        for frame in motion:
            self.take_substep(frame)
            yield path

    def crawl(self, step_length: float = 0.8, *, resolution: int = 50) -> Generator[Matrix]:
        feet = (0, 7, 6, 1)
        for foot in feet:
            locks = [(other_foot, slice(0,3)) for other_foot in feet if foot != other_foot]
            arc = partial(step_arc, d=step_length)
            step = steps.Step(foot, arc, locks)
            yield from self.take_step(step, resolution=resolution)
