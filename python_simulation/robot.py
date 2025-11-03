import numpy as np
from collections.abc import Callable, Generator
from itertools import pairwise

import cvxpy

from linalg import Matrix, Vector, unit_vector
from state import RobotState
from truss_config import Triangles, TrussConfig, edges, get_support_indices


def calc_rigidity_matrix(positions: Matrix, triangles: Triangles) -> Matrix:
    num_nodes, dim = positions.shape
    j = num_nodes * np.arange(dim)
    R = np.zeros((3*len(triangles), num_nodes*dim))
    for i, (e1, e2) in enumerate(edges(triangles)):
        u = unit_vector(positions[e1] - positions[e2])
        R[i, e1 + j] = u
        R[i, e2 + j] = -u
    return R


def calc_edge_lengths(positions: Matrix, triangles: Triangles) -> Matrix:
    return np.array([[
        np.linalg.norm(positions[i] - positions[j]) for i, j in edges(triangles)
    ]])


def calc_roll_to_length(num_triangles: int) -> Matrix:
    return np.kron(
        np.eye(num_triangles), np.array([[-1, 0], [1, -1], [0, 1]])
    )

def calc_length_to_roll(num_triangles: int) -> Matrix:
    return np.kron(
        np.eye(num_triangles), np.array([[-2, 1, 1,], [-1, -1, 2]]) / 3
    )


class Robot:
    def __init__(self, config: TrussConfig) -> None:
        self.config = config
        positions = config.initial_pos.copy()
        self.num_nodes, self.dim = positions.shape
        self.num_triangles = len(config.triangles)
        self.B_T = calc_roll_to_length(self.num_triangles)
        self.L2th = calc_length_to_roll(self.num_triangles)
        self.rigidity = calc_rigidity_matrix(positions, self.config.triangles)
        self.support_indices = get_support_indices(self.config)
        self.num_rollers = self.B_T.shape[1]
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

    @property
    def move_node_pos(self) -> Vector:
        return self.pos[self.config.move_node]

    def pos_of(self, node: int) -> Vector:
        return self.pos[node]

    def next_state_from_pos(self, d_pos: Vector) -> RobotState:
        d_roll = self.L2th @ self.rigidity @ d_pos
        d_pos_mat = d_pos.reshape(self.pos.shape, order='F')
        return RobotState(
            pos=self.pos + d_pos_mat,
            roll=self.roll + d_roll,
        )

    def next_state_from_roll(self, d_roll: Vector) -> RobotState:
        not_supports = [i for i in range(self.num_nodes*self.dim) if i not in self.support_indices]

        R_reduced = self.rigidity[:, not_supports]
        R_inv = np.linalg.inv(R_reduced)
        d_pos_reduced = R_inv @ self.B_T @ d_roll

        d_pos = np.zeros((self.num_nodes*self.dim,))
        d_pos[not_supports] = d_pos_reduced
        d_pos_mat = d_pos.reshape(self.pos.shape, order='F')

        return RobotState(
            pos=self.pos + d_pos_mat,
            roll=self.roll + d_roll,
        )

    def update_state(self, state: RobotState, t: float) -> None:
        self.t_hist.append(t)
        self.state_hist.append(state)
        self.rigidity = calc_rigidity_matrix(self.pos, self.config.triangles)

    def update_state_from_vel(self, vel: Vector, dt: float) -> None:
        state = self.next_state_from_pos(vel*dt)
        t = self.t_hist[-1] + dt
        self.update_state(state, t)

    def update_state_from_roll(self, roll: Vector, t: float) -> None:
        state = self.next_state_from_roll(roll - self.roll)
        self.update_state(state, t)

    def make_constraint_matrices(
        self,
        *,
        move_node: int | None = None,
        node_vel: Vector | None = None,
        broken_rollers: list[int] | None = None,
    ) -> tuple[Matrix, Vector]:
        dim = self.dim
        num_nodes = self.num_nodes

        A_move = np.zeros((dim, dim*num_nodes))
        i = np.arange(dim)
        if move_node is None:
            move_node = self.config.move_node
        A_move[i, move_node + i*num_nodes] = 1
        if node_vel is None:
            b_move = np.zeros((dim,))
        else:
            b_move = node_vel.reshape((dim,))

        n_lock = len(self.support_indices)
        A_lock = np.zeros((n_lock, dim*num_nodes))
        A_lock[np.arange(n_lock), self.support_indices] = 1
        b_lock = np.zeros((n_lock,))

        triangle_loops = np.kron(np.eye(self.num_triangles), np.ones((1, 3)))
        A_loop = triangle_loops @ self.rigidity
        b_loop = np.zeros((len(A_loop),))

        broken_rollers = [] if broken_rollers is None else broken_rollers
        vel2rollrate = self.L2th @ self.rigidity
        A_broken = vel2rollrate[broken_rollers]
        b_broken = np.zeros((len(broken_rollers),))

        A_payload = np.zeros((len(self.config.payload),num_nodes*dim))
        b_payload = np.zeros((len(self.config.payload),))
        j = num_nodes * np.arange(dim)
        for i, (e1,e2) in enumerate(self.config.payload):
            delta_pos = self.pos[e1]-self.pos[e2]
            A_payload[i, e1+j] = delta_pos
            A_payload[i, e2+j] = -delta_pos

        Aeq = np.vstack([A_move, A_lock, A_loop, A_broken,A_payload])
        beq = np.concat([b_move, b_lock, b_loop, b_broken, b_payload])
        return Aeq, beq

    def _get_opt_motion(self, node: int, node_vel: Vector) -> Vector:
        v = cvxpy.Variable(self.pos.size)
        A, b = self.make_constraint_matrices(move_node=node, node_vel=node_vel)
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
        ctrl_func: Callable[[Vector], Vector] = unit_vector,
    ) -> Generator[tuple[Vector, Vector]]:
        error = target - self.pos_of(node)
        while np.linalg.norm(error) > 0.01:
            node_vel = ctrl_func(error)
            yield self.pos_of(node), node_vel
            vel = self._get_opt_motion(node, node_vel)
            self.update_state_from_vel(vel, dt)
            error = target - self.pos_of(node)

    def move_node_along_path(self, node: int, path: Matrix) -> Generator[tuple[Vector, Vector]]:
        for point in path:
            yield from self.move_node_toward_pos(node, point)
