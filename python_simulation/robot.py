import numpy as np
from collections.abc import Callable, Generator, Iterable
from itertools import pairwise
from typing import Protocol

import cvxpy

from linalg import Matrix, Vector, unit_vector
from state import RobotState
from truss_config import Link, Lock, TrussConfig


def calc_rigidity_matrix(link: Iterable[Link], positions: Matrix) -> Matrix:
    dim = positions.shape[1]
    j = np.arange(dim)
    rows: list[Vector] = []
    for n1, n2 in link:
        row = np.zeros(positions.size)
        u = unit_vector(positions[n1] - positions[n2])
        row[dim*n1 + j] = u
        row[dim*n2 + j] = -u
        rows.append(row)
    return np.array(rows)


def calc_link_lengths(positions: Matrix, links: Iterable[Link]) -> Matrix:
    return np.array([[
        np.linalg.norm(positions[i] - positions[j]) for i, j in links
    ]])


def calc_roll_to_length(num_triangles: int) -> Matrix:
    return np.kron(
        np.eye(num_triangles), np.array([[-1, 0], [1, -1], [0, 1]])
    )

def calc_length_to_roll(num_triangles: int) -> Matrix:
    return np.kron(
        np.eye(num_triangles), np.array([[-2, 1, 1,], [-1, -1, 2]]) / 3
    )


def calc_length_constraints(
    num_triangles: int, *, broken_rollers: list[int] | None = None
) -> Matrix:
    A = np.kron(np.eye(num_triangles), np.ones((1, 3)))
    if broken_rollers:
        length_to_roll = calc_length_to_roll(num_triangles)
        A = np.vstack([A, length_to_roll[broken_rollers]])
    return A


def make_move_contraint(motion: Matrix) -> tuple[Matrix, Vector]:
    i = ~np.isnan(motion)
    b = motion[i]
    A = np.zeros((b.size, i.size))
    A[:, i.flat] = np.eye(len(A))
    return A, b


class Robot(Protocol):
    config: TrussConfig
    dim: int
    num_rollers: int
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
        positions = config.initial_pos.copy()
        self.num_nodes, self.dim = positions.shape
        self.B_T = calc_roll_to_length(len(config.triangles))
        self.rigidity = calc_rigidity_matrix(self.config.links, positions)
        self.num_rollers = self.B_T.shape[1]
        self.state = RobotState(
            pos=positions,
            roll=np.zeros((self.num_rollers,)),
        )

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

        d_pos = np.zeros((self.num_nodes*self.dim,))
        d_pos[not_supports] = d_pos_reduced
        d_pos_mat = d_pos.reshape(self.pos.shape)

        return RobotState(
            pos=self.pos + d_pos_mat,
            roll=self.roll + d_roll,
        )

    def update_state_from_roll(self, roll: Vector) -> None:
        self.state = self.next_state_from_roll(roll - self.roll)
        self.rigidity = calc_rigidity_matrix(self.config.links, self.pos)


class RobotInverse(RollHistRobot):
    def __init__(self, config: TrussConfig) -> None:
        self.config = config
        positions = config.initial_pos.copy()
        self.num_nodes, self.dim = positions.shape
        num_triangles = len(config.triangles)
        self.L2th = calc_length_to_roll(num_triangles)
        self.rigidity = calc_rigidity_matrix(self.config.links, positions)
        self.length_constraint = calc_length_constraints(num_triangles)
        self.num_rollers = self.L2th.shape[0]
        self.state_hist = [RobotState(
            pos=positions,
            roll=np.zeros((self.num_rollers,)),
        )]
        self.t_hist = [0.]

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
        self.rigidity = calc_rigidity_matrix(self.config.links, self.pos)

    def make_constraint_matrices(self, motion: Matrix) -> tuple[Matrix, Vector]:
        dim = self.dim
        num_nodes = self.num_nodes

        A_move, b_move = make_move_contraint(motion)

        A_length = self.length_constraint @ self.rigidity
        b_length = np.zeros((len(A_length),))

        A_payload = np.zeros((len(self.config.payload),num_nodes*dim))
        b_payload = np.zeros((len(self.config.payload),))
        j = np.arange(dim)
        for i, (e1,e2) in enumerate(self.config.payload):
            delta_pos = self.pos[e1]-self.pos[e2]
            A_payload[i, dim*e1 + j] = delta_pos
            A_payload[i, dim*e2 + j] = -delta_pos

        Aeq = np.vstack([A_move, A_length, A_payload])
        beq = np.concat([b_move, b_length, b_payload])
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
