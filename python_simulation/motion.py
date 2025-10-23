import numpy as np
from collections.abc import Callable, Generator
from dataclasses import dataclass, field

from scipy.optimize import minimize

import viz
from linalg import Matrix, Vector, unit_vector
from truss_robot import TrussRobot


def make_constraint_matrices(
    robot: TrussRobot,
    *,
    move_velocity: Vector | Matrix | None = None,
    broken_rollers: list[int] | None = None,
) -> tuple[Matrix, Matrix]:
    dim = robot.dim
    num_nodes = robot.num_nodes

    A_move = np.zeros((dim, dim*num_nodes))
    i = np.arange(dim)
    A_move[i, robot.config.move_node + i*num_nodes] = 1
    if move_velocity is None:
        b_move = np.zeros((dim, 1))
    else:
        b_move = move_velocity.reshape((dim, 1))

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


@dataclass(kw_only=True)
class MotionPlanner:
    robot: TrussRobot
    path: Matrix
    dt: float = 0.01
    curr_goal_idx: int = 0
    obj_str: str = 'Ldot'
    ctrl_func: Callable[[Vector], Vector] = unit_vector
    constraints: tuple[Matrix, Matrix] = field(init=False)

    def __post_init__(self) -> None:
        self.constraints = make_constraint_matrices(self.robot)

    @property
    def b_move(self) -> Vector:
        return self.constraints[1][0:self.robot.dim, 0]

    def _get_objective(self) -> tuple[Matrix, Matrix]:
        if self.obj_str == "Ldot":
            obj = self.robot.rigidity
            H = 2 * (obj.T @ obj)
            f = np.zeros((H.shape[0], 1))
        else:
            raise ValueError(f"{self.obj_str!r} is an invalid objective")
        return H, f

    def _get_opt_motion(self) -> Matrix:
        H, f = self._get_objective()
        def objective(xdot):
            return (0.5 * xdot.T @ H @ xdot + f.T @ xdot).ravel()

        Aeq, beq = self.constraints
        constraints = [{"type": "eq", "fun": lambda x: (Aeq @ x.T - beq.T).ravel()}]
        x0 = np.zeros_like(f).ravel()

        result = minimize(
            objective,
            x0,
            method="trust-constr",
            constraints=constraints,
            options={"xtol": 1e-8, "disp": False, "maxiter": 10000},
        )
        return 2*result.x.reshape(f.shape)

    def _get_error(self) -> Vector:
        target = self.path[self.curr_goal_idx]
        position = self.robot.move_node_pos
        return target - position

    def _refresh_constraints(self) -> None:
        error = self._get_error()
        self.constraints = make_constraint_matrices(
            self.robot, move_velocity=self.ctrl_func(error)
        )

    def _step_in_direction(self) -> Matrix:
        xd_opt = self._get_opt_motion()
        self._refresh_constraints()
        return xd_opt

    def move_ol(self) -> Generator[tuple[Vector, Vector]]:
        # Note that the first point in the path is the starting position.
        for i in range(1, len(self.path)):
            self.curr_goal_idx = i
            j = 0
            while np.linalg.norm(self._get_error()) > 0.01 and j < 10000:
                yield self.robot.move_node_pos, self.b_move
                xd_opt = self._step_in_direction()
                self.robot.update_state_from_vel(xd_opt, self.dt)
                j += 1

    def move_cl(self) -> Generator[tuple[Vector, Vector], tuple[Matrix, float], None]:
        count = 0
        while True:
            roll, t = yield self.robot.move_node_pos, self.b_move
            count += 1
            self.robot.update_state_from_roll(roll, t)
            self._refresh_constraints()


if __name__ == "__main__":
    import path, truss_config
    config_3d = truss_config.CONFIG_3D_1
    config_2d = truss_config.CONFIG_2D_1
    rover = truss_config.CONFIG_3D_ROVER1
    path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))
    path_2d = path.make_path(dimension=2)

    ol_robot = TrussRobot(rover)

    ol_planner = MotionPlanner(
        robot=ol_robot,
        path=ol_robot.move_node_pos + path_3d,
    )

    fig = viz.make_motion_fig(ol_robot, ol_planner.path)
    next(fig)
    for pos, vel in ol_planner.move_ol():
        fig.send((pos, vel))
