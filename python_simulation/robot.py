import numpy as np
from collections.abc import Callable, Generator
from itertools import pairwise

from scipy.optimize import NonlinearConstraint, minimize

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


class Robot:
    def __init__(self, config: TrussConfig) -> None:
        self.config = config
        positions = config.initial_pos.copy()
        self.num_nodes, self.dim = positions.shape
        self.num_triangles = len(config.triangles)
        self.B_T = calc_roll_to_length(self.num_triangles)
        self.L2th = np.linalg.pinv(self.B_T)
        self.rigidity = calc_rigidity_matrix(positions, self.config.triangles)
        self.support_indices = get_support_indices(self.config)
        self.num_rollers = self.B_T.shape[1]
        self.state_hist = [RobotState(
            pos=positions,
            roll=np.zeros((self.num_rollers, 1)),
        )]
        self.t_hist = [0.]

    @property
    def pos(self) -> Matrix:
        return self.state_hist[-1].pos

    @property
    def roll(self) -> Matrix:
        return self.state_hist[-1].roll

    @property
    def rollrate(self) -> Matrix:
        if len(self.state_hist) < 2:
            return np.zeros_like(self.roll)
        d_roll = self.state_hist[-1].roll - self.state_hist[-2].roll
        d_time = self.t_hist[-1] - self.t_hist[-2]
        return d_roll / d_time

    @property
    def roll_hist(self) -> list[Matrix]:
        return [s.roll for s in self.state_hist]

    @property
    def rollrate_hist(self) -> list[Matrix]:
        rollrates = [np.zeros_like(self.roll)]
        for (t1, t2), (s1, s2) in zip(pairwise(self.t_hist), pairwise(self.state_hist)):
            rollrates.append((s2.roll - s1.roll) / (t2 - t1))
        return rollrates

    @property
    def move_node_pos(self) -> Vector:
        return self.pos[self.config.move_node]

    def pos_of(self, node: int) -> Vector:
        return self.pos[node]

    def next_state_from_pos(self, d_pos: Matrix) -> RobotState:
        d_roll = self.L2th @ self.rigidity @ d_pos
        d_pos = d_pos.reshape(self.pos.shape, order='F')
        return RobotState(
            pos=self.pos + d_pos,
            roll=self.roll + d_roll,
        )

    def next_state_from_roll(self, d_roll: Matrix) -> RobotState:
        not_supports = [i for i in range(self.num_nodes*self.dim) if i not in self.support_indices]

        R_reduced = self.rigidity[:, not_supports]
        R_inv = np.linalg.inv(R_reduced)
        d_pos_reduced = R_inv @ self.B_T @ d_roll

        d_pos = np.zeros((self.num_nodes*self.dim, 1))
        d_pos[not_supports] = d_pos_reduced
        d_pos = d_pos.reshape(self.pos.shape, order='F')

        return RobotState(
            pos=self.pos + d_pos,
            roll=self.roll + d_roll,
        )

    def update_state(self, state: RobotState, t: float) -> None:
        self.t_hist.append(t)
        self.state_hist.append(state)
        self.rigidity = calc_rigidity_matrix(self.pos, self.config.triangles)

    def update_state_from_vel(self, vel: Matrix, dt: float) -> None:
        state = self.next_state_from_pos(vel*dt)
        t = self.t_hist[-1] + dt
        self.update_state(state, t)

    def update_state_from_roll(self, roll: Matrix, t: float) -> None:
        state = self.next_state_from_roll(roll - self.roll)
        self.update_state(state, t)

    def make_constraint_matrices(
        self,
        *,
        move_node: int | None = None,
        node_vel: Vector | Matrix | None = None,
        broken_rollers: list[int] | None = None,
    ) -> tuple[Matrix, Matrix]:
        dim = self.dim
        num_nodes = self.num_nodes

        A_move = np.zeros((dim, dim*num_nodes))
        i = np.arange(dim)
        if move_node is None:
            move_node = self.config.move_node
        A_move[i, move_node + i*num_nodes] = 1
        if node_vel is None:
            b_move = np.zeros((dim, 1))
        else:
            b_move = node_vel.reshape((dim, 1))

        n_lock = (dim-1) * 3
        A_lock = np.zeros((n_lock, dim*num_nodes))
        A_lock[np.arange(n_lock), self.support_indices] = 1
        b_lock = np.zeros((n_lock, 1))

        triangle_loops = np.kron(np.eye(self.num_triangles), np.ones((1, 3)))
        A_loop = triangle_loops @ self.rigidity
        b_loop = np.zeros((len(A_loop), 1))

        broken_rollers = [] if broken_rollers is None else broken_rollers
        vel2rollrate = self.L2th @ self.rigidity
        A_broken = vel2rollrate[broken_rollers]
        b_broken = np.zeros((len(broken_rollers), 1))

        A_payload = np.zeros((len(self.config.payload),num_nodes*dim))
        j = num_nodes * np.arange(dim)
        for i in range(len(self.config.payload)):
            e = self.config.payload[i]
            A_payload[i,e[0]+j] = 1
            A_payload[i, e[1] + j] = -1
        b_payload = np.zeros((len(self.config.payload), 1))

        Aeq = np.vstack([A_move, A_lock, A_loop, A_broken,A_payload])
        beq = np.vstack([b_move, b_lock, b_loop, b_broken, b_payload])
        return Aeq, beq

    def _get_opt_motion(self, node: int, node_vel: Vector) -> Matrix:
        H = 2 * (self.rigidity.T @ self.rigidity)
        f = np.zeros((len(H), 1))
        A, b = self.make_constraint_matrices(move_node=node, node_vel=node_vel)

        def objective(xdot: Vector) -> Vector:
            return (0.5 * xdot.T @ H @ xdot + f.T @ xdot).ravel()

        def constraint(xdot: Vector) -> Vector:
            return (A @ xdot.T - b.T).ravel()

        result = minimize(
            objective,
            np.zeros_like(f).ravel(),
            method="trust-constr",
            constraints=[NonlinearConstraint(constraint, 0, 0)],
            options={
                "xtol": 1e-8,
                "disp": False,
                "maxiter": 10000,
            },
        )
        return 2*result.x.reshape(f.shape)

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
