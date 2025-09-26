from truss_robot import TrussRobot, Robot3D, Robot2D
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from dataclasses import dataclass

from viz import MotionViz, PlotViz

@dataclass
class MotionParams:
    """
    MotionParams encapsulates parameters for motion planning and execution.

    Attributes:
        objective (str): The objective function to optimize during motion planning (default: "Ldot").
        broken_rollers (list[int] or None): List of indices representing broken rollers; None if all rollers are functional.
        dt (float): Time step for simulation or control loop in seconds (default: 0.001).
        animate_robot (bool): Whether to animate the robot during execution (default: True).
        refresh_rate (int): Number of simulation steps between animation frame updates (default: 10).
        verbose_print (bool): If True, enables detailed print statements for debugging (default: False).
    """
    objective: str = "Ldot"
    broken_rollers: list[int] = None
    dt: float = 0.001
    animate_robot: bool = True
    refresh_rate: int = 10
    verbose_print: bool = False

class MotionConstraintsGenerator:
    def __init__(self, robot: TrussRobot, broken_rollers: list[int] = None):
        """
        Initialize the motion object for a TrussRobot.
        Args:
            robot (TrussRobot): The truss robot instance containing geometry and configuration.
            broken_rollers (list[int], optional): List of indices for broken roller supports. Defaults to None.
        """

        self.robot = robot
        self.dim = self.robot.dim
        self.num_supports = len(self.robot.supports)
        self.target_node_idx = self.robot.move_node[0] - 1
        self.num_nodes = self.robot.num_nodes
        self.broken_rollers = broken_rollers
        self.A_move = None
        self.b_move = None
        self.A_lock = None
        self.b_lock = None
        self.A_loopcon = None
        self.b_loopcon = None
        self.A_broken = None
        self.b_broken = None
        self.Aeq = None
        self.beq = None
        self.constraint_indices_range: dict[str, tuple[int]] = {}

    def _calc_move_constraints(self):
        self.A_move = np.zeros((self.dim, self.dim * self.num_nodes))
        for i in range(self.dim):
            self.A_move[i, self.target_node_idx + i * self.num_nodes] = 1
        self.b_move = np.zeros((self.A_move.shape[0], 1))

        self.constraint_indices_range["move"] = (0, self.A_move.shape[0])

    def _calc_lock_constraints(self):
        n_lock = 6 if self.dim == 3 else 3
        self.A_lock = np.zeros((n_lock, self.dim * self.num_nodes))
        last_row_updated = -1
        for i, support in enumerate(self.robot.supports):
            support_index = support - 1
            for j in range(i, self.dim):
                last_row_updated += 1
                self.A_lock[last_row_updated, support_index + j * self.num_nodes] = 1

        self.b_lock = np.zeros((self.A_lock.shape[0], 1))

        self.constraint_indices_range["lock"] = (
            self.constraint_indices_range["move"][1],
            self.constraint_indices_range["move"][1] + self.A_lock.shape[0],
        )

    def _calc_loop_constraints(self):
        self.A_loopcon = self.robot.triangle_loops @ self.robot.R
        self.b_loopcon = np.zeros((self.A_loopcon.shape[0], 1))
        self.constraint_indices_range["loop"] = (
            self.constraint_indices_range["lock"][1],
            self.constraint_indices_range["lock"][1] + self.A_loopcon.shape[0],
        )

    def _calc_broken_constraints(self):
        thetadot = self.robot.L2th @ self.robot.R
        self.A_broken = thetadot[[node - 1 for node in self.broken_rollers], :]
        self.b_broken = np.zeros((self.A_broken.shape[0], 1))
        self.constraint_indices_range["broken"] = (
            self.constraint_indices_range["loop"][1],
            self.constraint_indices_range["loop"][1] + self.A_broken.shape[0],
        )

    def _calc_unit_vector(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm

    def _update_loopcon(self):
        self.A_loopcon = self.robot.triangle_loops @ self.robot.R
        start, stop = self.constraint_indices_range["loop"]
        self.Aeq[start:stop, :] = self.A_loopcon

    def _update_broken_constraints(self):
        self._calc_broken_constraints()
        start, stop = self.constraint_indices_range["broken"]
        self.Aeq[start:stop, :] = self.A_broken
        self.beq[start:stop, :] = self.b_broken

    def _update_b_move(self, dir: np.ndarray):
        self.b_move = self._calc_unit_vector(dir.reshape(self.b_move.shape))

        start, stop = self.constraint_indices_range["move"]
        self.beq[start:stop, :] = self.b_move

    def calc_init_constraints(self):
        As = []
        bs = []

        self._calc_move_constraints()
        As.append(self.A_move)
        bs.append(self.b_move)

        self._calc_lock_constraints()
        As.append(self.A_lock)
        bs.append(self.b_lock)

        self._calc_loop_constraints()
        As.append(self.A_loopcon)
        bs.append(self.b_loopcon)

        if self.broken_rollers is not None:
            self._calc_broken_constraints()
            As.append(self.A_broken)
            bs.append(self.b_broken)

        self.Aeq = np.block([[A] for A in As])
        self.beq = np.block([[b] for b in bs])

    def update_constraint_matrices(self, dir: np.ndarray):
        self._update_loopcon()
        self._update_b_move(dir)
        if self.broken_rollers is not None:
            self._update_broken_constraints()

    def get_constraint_matrices(self):
        return self.Aeq, self.beq

    def get_b_move(self):
        return self.b_move

class MotionPlanner:
    def __init__(self, robot: TrussRobot, motion_params: MotionParams):
        """
        Initializes the motion object with robot configuration and motion parameters.
        Args:
            robot (TrussRobot): The robot object containing geometry and configuration.
            motion_params (MotionParams): Parameters specifying motion settings, such as broken rollers, time step, and objective.
        """

        self.robot = robot
        self.dim = self.robot.dim
        self.num_supports = len(self.robot.supports)
        self.target_node_idx = self.robot.move_node[0] - 1
        self.num_nodes = self.robot.num_nodes
        self.broken_rollers = motion_params.broken_rollers
        self.dt = motion_params.dt
        self.curr_goal_idx = 0
        self.curr_goal = None
        self.goal_direction = None
        self.count = 0

        self.obj_str = motion_params.objective

        self.motion_viz: MotionViz = None

        self.motion_constraints_generator = MotionConstraintsGenerator(self.robot, self.broken_rollers)

        self.motion_constraints_generator.calc_init_constraints()

        np.set_printoptions(precision=4, suppress=True)

    def _get_objective(self, obj_str):
        if obj_str == "Ldot":
            obj = self.robot.R
            H = 2 * (obj.T @ obj)
            f = np.zeros((H.shape[0], 1))
        else:
            raise Exception("Invalid objective specified")

        return H, f

    def _solve_quadprog(self, H, f):
        def objective(xdot):
            return (0.5 * xdot.T @ H @ xdot + f.T @ xdot).ravel()

        constraints = []

        Aeq, beq = self.motion_constraints_generator.get_constraint_matrices()
        constraints.append(
            {"type": "eq", "fun": lambda x: (Aeq @ x.T - beq.T).ravel()}
        )

        x0 = np.zeros_like(f).ravel()

        result = minimize(
            objective,
            x0,
            method="trust-constr",
            constraints=constraints,
            options={"xtol": 1e-8, "disp": False, "maxiter": 10000},
        )

        return result.x.reshape(f.shape), result.success, result.status, result.message

    def _get_opt_motion(self):
        self.robot.R = self.robot.get_R()
        H, f = self._get_objective(self.obj_str)
        xd_opt, success, status, message = self._solve_quadprog(H, f)

        return xd_opt, self.robot.R @ xd_opt

    def _calc_goal_direction(self):
        self.curr_goal = self.robot.path.transformed_path[self.curr_goal_idx]
        move_node_position = self.robot.get_move_nodes_pos()
        goal_direction = self.curr_goal - move_node_position
        return goal_direction

    def _ol_step_in_direction(self, t):
        xd_opt, Ldot = self._step_in_direction()

        self.robot.ol_update_and_store_positions_and_R(t, self.dt, xd_opt, Ldot)

        if self.motion_viz:
            self.motion_viz.update_plot()

    def _cl_step_in_direction(self):
        xd_opt, Ldot = self._step_in_direction()

        if self.motion_viz:
            self.motion_viz.update_plot()

        return xd_opt, Ldot

    def _step_in_direction(self):
        xd_opt, Ldot = self._get_opt_motion()

        self.goal_direction = self._calc_goal_direction()

        self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)

        return xd_opt, Ldot

    def _print_debug_info(self):
        print(f"Current Goal Index: {self.curr_goal_idx}")
        print(f"Current Goal Position: {self.curr_goal}")
        move_node_position = self.robot.get_move_nodes_pos()
        print(f"Move Node Position: {move_node_position}")
        print(f"Goal Direction: {self.goal_direction}")
        print(f"Goal Direction Norm: {np.linalg.norm(self.goal_direction)}")
        b_move = self.motion_constraints_generator.get_b_move()
        print(f"b_move: {b_move.ravel()}")
        print(f"Theta: {self.robot.theta_hist[-1].ravel()}")
        print(f"Theta dot: {self.robot.thetad_hist[-1].ravel()}")
        print(f"Perimeters: {self.robot.get_perimeters()}")
        print(f"Perimeter Difference: {self.robot.get_perimeters_diff()}")
        print("------------------------------------------------")

    def move_ol(self, motion_params: MotionParams) -> tuple[np.typing.NDArray, TrussRobot]:
        '''Open loop motion of the robot along its path'''
        t = 0
        animate_robot = motion_params.animate_robot
        refresh_rate = motion_params.refresh_rate
        verbose_print = motion_params.verbose_print

        if animate_robot:
            self.motion_viz = MotionViz(self.robot, refresh_rate=refresh_rate)

        for _ in range(len(self.robot.path.transformed_path)):
            if self.curr_goal_idx == 0:
                self.curr_goal_idx += 1
                continue  # Because the first point in the path is the starting position

            self.goal_direction = self._calc_goal_direction()
            self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)
            self.count = 0

            while np.linalg.norm(self.goal_direction) > 0.01 and self.count < 10000:
                if self.motion_viz and self.count % refresh_rate == 0:
                    self.motion_viz.update_motion_coords(
                        move_node_position=self.robot.get_move_nodes_pos(), b_move=self.motion_constraints_generator.get_b_move()
                    )

                self._ol_step_in_direction(t)

                if verbose_print and self.count % refresh_rate == 0:
                    self._print_debug_info()

                t += self.dt
                self.count += 1

            self.curr_goal_idx += 1

        return self.robot.thetad_hist, self.robot

    def move_cl(self, motion_params: MotionParams, t, dt, thetas: np.ndarray) -> tuple[np.typing.NDArray, TrussRobot]:
        animate_robot = motion_params.animate_robot
        refresh_rate = motion_params.refresh_rate
        verbose_print = motion_params.verbose_print

        finished = False

        self.robot.fk_position(t, dt, thetas)

        if self.goal_direction is None:
            self.goal_direction = self._calc_goal_direction()
            self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)
            if animate_robot:
                self.motion_viz = MotionViz(self.robot, refresh_rate=refresh_rate)

        if np.linalg.norm(self.goal_direction) < 0.01:
            self.curr_goal_idx += 1
            if self.curr_goal_idx >= len(self.robot.path.transformed_path):
                print("Already at end of path")
                finished = True
                return np.zeros((1, self.robot.num_rollers)), self.robot, finished

        if self.motion_viz and self.count % refresh_rate == 0:
            self.motion_viz.update_motion_coords(
                move_node_position=self.robot.get_move_nodes_pos(), b_move=self.motion_constraints_generator.get_b_move()
            )

        self.goal_direction = self._calc_goal_direction()
        self.motion_constraints_generator.update_constraint_matrices(self.goal_direction)

        xd_opt, _ = self._cl_step_in_direction()

        thetad_opt = self.robot.convert_xd_to_thetad(xd_opt)

        self.count += 1
        if verbose_print and self.count % refresh_rate == 0:
            self._print_debug_info()

        return thetad_opt, self.robot, finished


if __name__ == "__main__":
    # ol_robot = Robot3D(1, RPYrot=[90., -45.0, 45.0], path_scale=1, path_type="polygon", num_sides=4)
    ol_robot = Robot2D(1, RPYrot=[0], path_scale=1, path_type="polygon", num_sides=4)

    motion_params = MotionParams(animate_robot=True, refresh_rate=10,
                                 verbose_print=False, objective="Ldot",
                                 broken_rollers=None, dt=0.01)

    ol_planner = MotionPlanner(robot=ol_robot, motion_params=motion_params)

    thetads, ol_robot = ol_planner.move_ol(motion_params=motion_params)