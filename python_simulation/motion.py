import numpy as np
from collections.abc import Callable
from dataclasses import dataclass, field

from scipy.optimize import minimize

from linalg import Matrix, Vector, unit_vector
from truss_robot import TrussRobot, calc_edge_lengths
from viz import MotionViz


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
    motion_viz: MotionViz | None = None
    constraints: tuple[Matrix, Matrix] = field(init=False)

    def __post_init__(self) -> None:
        self.constraints = make_constraint_matrices(self.robot)
        if self.motion_viz is not None:
            self.motion_viz.init_animation(self.path)

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
        if self.motion_viz:
            self.motion_viz.update_plot()
        return xd_opt

    def _print_debug_info(self) -> None:
        print(f"Current Goal Index: {self.curr_goal_idx}")
        print(f"Current Goal Position: {self.path[self.curr_goal_idx]}")
        move_node_position = self.robot.move_node_pos
        print(f"Move Node Position: {move_node_position}")
        print(f"Goal Direction: {self._get_error()}")
        print(f"Goal Direction Norm: {np.linalg.norm(self._get_error())}")
        print(f"b_move: {self.b_move.ravel()}")
        print(f"Roll dist: {self.robot.roll.ravel()}")
        print(f"Roll rate: {self.robot.rollrate.ravel()}")
        p0 = sum(calc_edge_lengths(self.robot.state_hist[0].pos, self.robot.config.triangles))
        p1 = sum(calc_edge_lengths(self.robot.pos, self.robot.config.triangles))
        print(f"Perimeter: {p1}")
        print(f"Perimeter Difference: {p1 - p0}")
        print("------------------------------------------------")

    def move_ol(self, *, verbose_print_rate: int = 0) -> None:
        '''Open loop motion of the robot along its path'''
        for count in range(len(self.path)):
            if self.curr_goal_idx == 0:
                self.curr_goal_idx += 1
                continue  # Because the first point in the path is the starting position

            while np.linalg.norm(self._get_error()) > 0.01 and count < 10000:
                if self.motion_viz and count % self.motion_viz.refresh_rate == 0:
                    self.motion_viz.update_motion_coords(
                        move_node_position=self.robot.move_node_pos, b_move=self.b_move
                    )

                xd_opt = self._step_in_direction()
                self.robot.update_state_from_vel(xd_opt, self.dt)

                if verbose_print_rate and count % verbose_print_rate == 0:
                    self._print_debug_info()

            self.curr_goal_idx += 1

    def move_cl(self, thetas: Matrix, t: float, *, verbose_print_rate: int = 0) -> bool:
        self.robot.update_state_from_roll(thetas, t)
        self._refresh_constraints()

        if np.linalg.norm(self._get_error()) < 0.01:
            self.curr_goal_idx += 1
            if self.curr_goal_idx >= len(self.path):
                print("Already at end of path")
                return True

        if self.motion_viz and self.count % self.motion_viz.refresh_rate == 0:
            self.motion_viz.update_motion_coords(
                move_node_position=self.robot.move_node_pos, b_move=self.b_move
            )

        self.count += 1
        if verbose_print_rate and self.count % verbose_print_rate == 0:
            self._print_debug_info()

        return False


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
        motion_viz = MotionViz(ol_robot),
    )

    ol_planner.move_ol()
