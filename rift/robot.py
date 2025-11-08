import numpy as np
from collections.abc import Callable, Generator, Iterable
from itertools import pairwise
from typing import Protocol

import cvxpy

from . import tubetruss
from .linalg import Matrix, Vector, unit_vector
from .state import RobotState
from .truss_config import Lock, TrussConfig


def make_move_contraint(motion: Matrix) -> tuple[Matrix, Vector]:
    i = ~np.isnan(motion)
    b = motion[i]
    A = np.zeros((b.size, i.size))
    A[:, i.flat] = np.eye(len(A))
    return A, b


class Robot(Protocol):
    config: TrussConfig
    @property
    def dim(self) -> int: ...
    @property
    def pos(self) -> Matrix: ...
    @property
    def roll(self) -> Vector: ...


class RollHistRobot(Robot):
    t_hist: list[float]
    @property
    def roll_hist(self) -> list[Vector]: ...
    @property
    def rollrate_hist(self) -> list[Vector]: ...


class RobotForward(Robot):
    def __init__(self, config: TrussConfig) -> None:
        self.config = config
        self.structure = config.triangles + config.payload
        positions = config.initial_pos.copy()
        self.B_T = tubetruss.get_incidence(self.structure)
        self.state = RobotState(pos=positions, roll=np.zeros(self.B_T.shape[1]))
        self.rigidity = tubetruss.get_rigidity(self.structure, self.state)

    @property
    def dim(self) -> int:
        return self.state.pos.shape[1]

    @property
    def pos(self) -> Matrix:
        return self.state.pos

    @property
    def roll(self) -> Vector:
        return self.state.roll

    def pos_of(self, node: int) -> Vector:
        return self.pos[node]

    def next_state_from_roll(self, d_roll: Vector) -> RobotState:
        s = np.ones_like(self.pos, dtype=np.bool)
        for lock in self.config.locks:
            s[lock] = False
        not_supports = np.flatnonzero(s)

        R_reduced = self.rigidity[:, not_supports]
        R_inv = np.linalg.inv(R_reduced)
        d_pos_reduced = R_inv @ self.B_T @ d_roll

        d_pos = np.zeros(self.pos.size)
        d_pos[not_supports] = d_pos_reduced
        d_pos_mat = d_pos.reshape(self.pos.shape)

        return RobotState(
            pos=self.pos + d_pos_mat,
            roll=self.roll + d_roll,
        )

    def update_state_from_roll(self, roll: Vector) -> None:
        self.state = self.next_state_from_roll(roll - self.roll)
        self.rigidity = tubetruss.get_rigidity(self.structure, self.state)


class RobotInverse(RollHistRobot):
    def __init__(self, config: TrussConfig) -> None:
        self.config = config
        self.structure = config.triangles + config.payload
        positions = config.initial_pos.copy()
        self.L2th = tubetruss.get_incidence_inv(self.structure)
        self.length_constraint = tubetruss.get_length_constraint(self.structure)
        self.state_hist = [RobotState(pos=positions, roll=np.zeros(self.L2th.shape[0]))]
        self.t_hist = [0.]
        self.rigidity = tubetruss.get_rigidity(self.structure, self.state_hist[0])

    @property
    def dim(self) -> int:
        return self.state_hist[-1].pos.shape[1]

    @property
    def pos(self) -> Matrix:
        return self.state_hist[-1].pos

    @property
    def roll(self) -> Vector:
        return self.state_hist[-1].roll

    @property
    def rollrate(self) -> Vector:
        if len(self.state_hist) < 2:
            return np.zeros_like(self.roll)
        d_roll = self.state_hist[-1].roll - self.state_hist[-2].roll
        d_time = self.t_hist[-1] - self.t_hist[-2]
        return d_roll / d_time

    @property
    def roll_hist(self) -> list[Vector]:
        return [s.roll for s in self.state_hist]

    @property
    def rollrate_hist(self) -> list[Vector]:
        rollrates = [np.zeros_like(self.roll)]
        for (t1, t2), (s1, s2) in zip(pairwise(self.t_hist), pairwise(self.state_hist)):
            rollrates.append((s2.roll - s1.roll) / (t2 - t1))
        return rollrates

    def pos_of(self, node: int) -> Vector:
        return self.pos[node]

    def next_state_from_pos(self, d_pos: Vector) -> RobotState:
        d_roll = self.L2th @ self.rigidity @ d_pos
        d_pos_mat = d_pos.reshape(self.pos.shape)
        return RobotState(
            pos=self.pos + d_pos_mat,
            roll=self.roll + d_roll,
        )

    def update_state_from_vel(self, vel: Vector, dt: float) -> None:
        state = self.next_state_from_pos(vel*dt)
        t = self.t_hist[-1] + dt
        self.t_hist.append(t)
        self.state_hist.append(state)
        self.rigidity = tubetruss.get_rigidity(self.structure, state)

    def make_constraint_matrices(self, motion: Matrix) -> tuple[Matrix, Vector]:
        A_move, b_move = make_move_contraint(motion)

        A_length = self.length_constraint @ self.rigidity
        b_length = np.zeros((len(A_length),))

        Aeq = np.vstack([A_move, A_length])
        beq = np.concat([b_move, b_length])
        return Aeq, beq

    def get_optimal_motion(self, motion: Matrix) -> Vector:
        v = cvxpy.Variable(self.pos.size)
        A, b = self.make_constraint_matrices(motion)
        cost = cvxpy.sum_squares(self.rigidity @ v)
        prob = cvxpy.Problem(cvxpy.Minimize(cost), [A @ v == b])
        prob.solve()
        assert v.value is not None
        return v.value

    def move_node_toward_pos(
        self,
        node: int,
        target: Vector,
        *,
        dt: float = 0.01,
        locks: Iterable[Lock] = (),
        ctrl_func: Callable[[Vector], Vector] = unit_vector,
    ) -> Generator[tuple[Vector, Vector]]:
        motion = np.full_like(self.pos, np.nan)
        for lock in locks:
            motion[lock] = 0.
        while np.linalg.norm(error := target - self.pos_of(node)) > 0.01:
            node_vel = ctrl_func(error)
            yield self.pos_of(node), node_vel
            motion[node] = node_vel
            vel = self.get_optimal_motion(motion)
            self.update_state_from_vel(vel, dt)

    def move_node_along_path(self, node: int, path: Matrix) -> Generator[tuple[Vector, Vector]]:
        for point in path:
            yield from self.move_node_toward_pos(node, point, locks=self.config.locks)
