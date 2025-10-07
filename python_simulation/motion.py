from truss_robot import TrussRobot
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from dataclasses import dataclass
from linalg import Vector, Matrix, unit_vector

from viz import MotionViz


class MotionConstraintsGenerator:
    def __init__(self, robot: TrussRobot, broken_rollers: list[int] | None = None) -> None:
        """
        Initialize the motion object for a TrussRobot.
        Args:
            robot (TrussRobot): The truss robot instance containing geometry and configuration.
            broken_rollers (list[int], optional): List of indices for broken roller supports. Defaults to an empty list.
        """
        self.robot = robot
        dim = self.robot.dim
        num_nodes = self.robot.num_nodes
        self.broken_rollers = [] if broken_rollers is None else broken_rollers

        target_node_idx = self.robot.move_node
        self.A_move = np.zeros((dim, dim * num_nodes))
        for i in range(dim):
            self.A_move[i, target_node_idx + i*num_nodes] = 1
        self.b_move = np.zeros((dim, 1))

        n_lock = (dim-1) * 3
        self.A_lock = np.zeros((n_lock, dim*num_nodes))
        last_row_updated = -1
        for i, support in enumerate(self.robot.supports):
            for j in range(i, dim):
                last_row_updated += 1
                self.A_lock[last_row_updated, support + j*num_nodes] = 1
        self.b_lock = np.zeros((n_lock, 1))

        self.A_loopcon = self.robot.triangle_loops @ self.robot.rigidity
        self.b_loopcon = np.zeros((len(self.A_loopcon), 1))

        thetadot = self.robot.L2th @ self.robot.rigidity
        self.A_broken = thetadot[[node - 1 for node in self.broken_rollers], :]
        self.b_broken = np.zeros((len(self.broken_rollers), 1))

    def update_constraint_matrices(self, dir: Vector | Matrix) -> None:
        self.A_loopcon = self.robot.triangle_loops @ self.robot.rigidity
        self.b_move = unit_vector(dir.reshape(self.b_move.shape))
        thetadot = self.robot.L2th @ self.robot.rigidity
        self.A_broken = thetadot[[node - 1 for node in self.broken_rollers], :]
        self.b_broken = np.zeros((self.A_broken.shape[0], 1))

    def get_constraint_matrices(self) -> tuple[Matrix, Matrix]:
        Aeq = np.vstack([self.A_move, self.A_lock, self.A_loopcon, self.A_broken])
        beq = np.vstack([self.b_move, self.b_lock, self.b_loopcon, self.b_broken])
        return Aeq, beq

    def get_b_move(self) -> Vector | Matrix:
        return self.b_move


@dataclass(kw_only=True)
class MotionPlanner:
    robot: TrussRobot
    dt: float = 0.01
    curr_goal_idx: int = 0
    goal_direction: Vector | None = None
    obj_str: str = 'Ldot'
    motion_viz: MotionViz | None = None
    motion_constraints_generator: MotionConstraintsGenerator

    def _get_objective(self) -> tuple[Matrix, Matrix]:
        if self.obj_str == "Ldot":
            obj = self.robot.rigidity
            H = 2 * (obj.T @ obj)
            f = np.zeros((H.shape[0], 1))
        else:
            raise ValueError(f"{self.obj_str!r} is an invalid objective")
        return H, f

    def _get_opt_motion(self) -> tuple[Matrix, Matrix]:
        H, f = self._get_objective()
        def objective(xdot):
            return (0.5 * xdot.T @ H @ xdot + f.T @ xdot).ravel()

        Aeq, beq = self.motion_constraints_generator.get_constraint_matrices()
        constraints = [{"type": "eq", "fun": lambda x: (Aeq @ x.T - beq.T).ravel()}]
        x0 = np.zeros_like(f).ravel()

        result = minimize(
            objective,
            x0,
            method="trust-constr",
            constraints=constraints,
            options={"xtol": 1e-8, "disp": False, "maxiter": 10000},
        )

        xd_opt = result.x.reshape(f.shape)
        return xd_opt, self.robot.rigidity @ xd_opt

    def _calc_goal_direction(self) -> Vector:
        curr_goal = self.robot.path[self.curr_goal_idx]
        move_node_position = self.robot.move_node_pos
        goal_direction = curr_goal - move_node_position
        return goal_direction

    def _ol_step_in_direction(self, t: float) -> None:
        xd_opt, Ldot = self._step_in_direction()
        self.robot.ol_update_and_store_positions_and_rigidity(t, self.dt, xd_opt, Ldot)
        if self.motion_viz:
            self.motion_viz.update_plot()

    def _cl_step_in_direction(self) -> tuple[Matrix, Matrix]:
        xd_opt, Ldot = self._step_in_direction()
        if self.motion_viz:
            self.motion_viz.update_plot()
        return xd_opt, Ldot

    def _step_in_direction(self) -> tuple[Matrix, Matrix]:
        xd_opt, Ldot = self._get_opt_motion()
        self.goal_direction = self._calc_goal_direction()
        self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)
        return xd_opt, Ldot

    def _print_debug_info(self) -> None:
        print(f"Current Goal Index: {self.curr_goal_idx}")
        print(f"Current Goal Position: {self.robot.path[self.curr_goal_idx]}")
        move_node_position = self.robot.move_node_pos
        print(f"Move Node Position: {move_node_position}")
        print(f"Goal Direction: {self.goal_direction}")
        if self.goal_direction is not None:
            print(f"Goal Direction Norm: {np.linalg.norm(self.goal_direction)}")
        b_move = self.motion_constraints_generator.get_b_move()
        print(f"b_move: {b_move.ravel()}")
        print(f"Theta: {self.robot.theta_hist[-1].ravel()}")
        print(f"Theta dot: {self.robot.thetad_hist[-1].ravel()}")
        print(f"Perimeter: {sum(self.robot.L_hist[-1])}")
        print(f"Perimeter Difference: {sum(self.robot.L_hist[-1]) - sum(self.robot.L_hist[0])}")
        print("------------------------------------------------")

    def move_ol(self, *, verbose_print_rate: int = 0) -> tuple[list[Matrix], TrussRobot]:
        '''Open loop motion of the robot along its path'''
        t = 0

        for _ in range(len(self.robot.path)):
            if self.curr_goal_idx == 0:
                self.curr_goal_idx += 1
                continue  # Because the first point in the path is the starting position

            self.goal_direction = self._calc_goal_direction()
            self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)
            count = 0

            while np.linalg.norm(self.goal_direction) > 0.01 and count < 10000:
                if self.motion_viz and count % self.motion_viz.refresh_rate == 0:
                    self.motion_viz.update_motion_coords(
                        move_node_position=self.robot.move_node_pos,
                        b_move=self.motion_constraints_generator.get_b_move()
                    )

                self._ol_step_in_direction(t)

                if verbose_print_rate and count % verbose_print_rate == 0:
                    self._print_debug_info()

                t += self.dt
                count += 1

            self.curr_goal_idx += 1

        return self.robot.thetad_hist, self.robot

    def move_cl(self, t: float, dt: float, thetas: Matrix, *, verbose_print_rate: int = 0) -> tuple[Matrix, TrussRobot, bool]:
        self.robot.fk_position(t, dt, thetas)

        if self.goal_direction is None:
            self.goal_direction = self._calc_goal_direction()
            self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)

        if np.linalg.norm(self.goal_direction) < 0.01:
            self.curr_goal_idx += 1
            if self.curr_goal_idx >= len(self.robot.path):
                print("Already at end of path")
                return np.zeros((1, self.robot.num_rollers)), self.robot, True

        if self.motion_viz and self.count % self.motion_viz.refresh_rate == 0:
            self.motion_viz.update_motion_coords(
                move_node_position=self.robot.move_node_pos,
                b_move=self.motion_constraints_generator.get_b_move(),
            )

        self.goal_direction = self._calc_goal_direction()
        self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)

        xd_opt, _ = self._cl_step_in_direction()

        thetad_opt = self.robot.convert_xd_to_thetad(xd_opt)

        self.count += 1
        if verbose_print_rate and self.count % verbose_print_rate == 0:
            self._print_debug_info()

        return thetad_opt, self.robot, False


if __name__ == "__main__":
    import path, truss_config
    config_3d = truss_config.CONFIG_3D_1
    config_2d = truss_config.CONFIG_2D_1
    path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))
    path_2d = path.make_path(dimension=2)

    ol_robot = TrussRobot(config_2d, path_2d)

    ol_planner = MotionPlanner(
        robot=ol_robot,
        motion_viz = MotionViz(ol_robot),
        motion_constraints_generator=MotionConstraintsGenerator(ol_robot),
    )

    thetads, ol_robot = ol_planner.move_ol()
