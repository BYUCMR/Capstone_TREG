import numpy as np
from collections.abc import Callable, Generator
from dataclasses import dataclass

from scipy.optimize import minimize

from linalg import Matrix, Vector, unit_vector
from truss_robot import TrussRobot


def make_constraint_matrices(
    robot: TrussRobot,
    *,
    move_node: int | None = None,
    node_vel: Vector | Matrix | None = None,
    broken_rollers: list[int] | None = None,
) -> tuple[Matrix, Matrix]:
    dim = robot.dim
    num_nodes = robot.num_nodes

    A_move = np.zeros((dim, dim*num_nodes))
    i = np.arange(dim)
    if move_node is None:
        move_node = robot.config.move_node
    A_move[i, move_node + i*num_nodes] = 1
    if node_vel is None:
        b_move = np.zeros((dim, 1))
    else:
        b_move = node_vel.reshape((dim, 1))

    n_lock = (dim-1) * 3
    A_lock = np.zeros((n_lock, dim*num_nodes))
    A_lock[np.arange(n_lock), robot.support_indices] = 1
    b_lock = np.zeros((n_lock, 1))

    triangle_loops = np.kron(np.eye(robot.num_triangles), np.ones((1, 3)))
    A_loop = triangle_loops @ robot.rigidity
    b_loop = np.zeros((len(A_loop), 1))

    broken_rollers = [] if broken_rollers is None else broken_rollers
    vel2rollrate = robot.L2th @ robot.rigidity
    A_broken = vel2rollrate[broken_rollers]
    b_broken = np.zeros((len(broken_rollers), 1))

    A_payload = np.zeros((len(robot.config.payload),num_nodes*dim))
    j = num_nodes * np.arange(dim)
    for i in range(len(robot.config.payload)):
        e = robot.config.payload[i]
        A_payload[i,e[0]+j] = 1
        A_payload[i, e[1] + j] = -1
    b_payload = np.zeros((len(robot.config.payload), 1))

    Aeq = np.vstack([A_move, A_lock, A_loop, A_broken,A_payload])
    beq = np.vstack([b_move, b_lock, b_loop, b_broken, b_payload])
    return Aeq, beq


@dataclass
class MotionPlanner:
    robot: TrussRobot

    def _get_opt_motion(self, node: int, node_vel: Vector) -> Matrix:
        H = 2 * (self.robot.rigidity.T @ self.robot.rigidity)
        f = np.zeros((len(H), 1))
        def objective(xdot):
            return (0.5 * xdot.T @ H @ xdot + f.T @ xdot).ravel()

        A, b = make_constraint_matrices(
            self.robot, move_node=node, node_vel=node_vel
        )
        constraints = [{"type": "eq", "fun": lambda x: (A @ x.T - b.T).ravel()}]
        x0 = np.zeros_like(f).ravel()

        result = minimize(
            objective,
            x0,
            method="trust-constr",
            constraints=constraints,
            options={"xtol": 1e-8, "disp": False, "maxiter": 10000},
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
        error = target - self.robot.pos_of(node)
        while np.linalg.norm(error) > 0.01:
            node_vel = ctrl_func(error)
            yield self.robot.pos_of(node), node_vel
            vel = self._get_opt_motion(node, node_vel)
            self.robot.update_state_from_vel(vel, dt)
            error = target - self.robot.pos_of(node)

    def move_node_along_path(self, node: int, path: Matrix) -> Generator[tuple[Vector, Vector]]:
        for point in path:
            yield from self.move_node_toward_pos(node, point)
