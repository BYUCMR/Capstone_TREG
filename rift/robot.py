import numpy as np
from collections.abc import Callable, Generator, Iterable
from itertools import pairwise
from typing import Protocol

import qpsolvers

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
        self.incidence = tubetruss.get_incidence(self.structure)
        self.state = RobotState(
            pos=config.initial_pos.copy(),
            roll=np.zeros(self.incidence.shape[1]),
        )

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

    def update_state(self, roll: Vector, *, locks: Iterable[Lock] = ()) -> None:
        can_move = np.ones_like(self.pos, dtype=np.bool)
        for lock in locks:
            can_move[lock] = False
        unlocked_indices = np.flatnonzero(can_move)

        R = tubetruss.get_rigidity(self.structure, self.state)
        R_reduced = R[:, unlocked_indices]
        R_inv = np.linalg.inv(R_reduced)
        d_pos_reduced = R_inv @ self.incidence @ (roll - self.roll)

        d_pos = np.zeros_like(self.pos)
        d_pos.put(unlocked_indices, d_pos_reduced)

        self.state = RobotState(roll=roll, pos=self.pos + d_pos)


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
    def state(self) -> RobotState:
        return self.state_hist[-1]

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
        if self.config.keep_level is not None:
            A_level = np.zeros((1, self.pos.size))
            A_level[0, 3*self.config.keep_level[0]+2] =  1
            A_level[0, 3*self.config.keep_level[1]+2] = -1
            A_move = np.vstack([A_move, A_level])
            b_move = np.concat([b_move, [0.]])

        A_length = self.length_constraint @ self.rigidity
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

    def crawl(self, angle: float = 0) -> Generator[Matrix]:
        step_up = np.array([np.cos(angle), np.sin(angle), 1.])
        step_down = np.array([np.cos(angle), np.sin(angle), -1.])
        step_forward = step_up + step_down
        feet = (0, 7, 6, 1)
        for foot in feet:
            locks = [(other_foot, slice(0,3)) for other_foot in feet if foot != other_foot]
            path = np.vstack([
                self.pos_of(foot),
                self.pos_of(foot)+step_up,
                self.pos_of(foot)+step_forward,
            ])
            for point in path:
                for _ in self.move_node_toward_pos(foot, point, locks=locks):
                    yield path
