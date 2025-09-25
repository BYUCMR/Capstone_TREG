from truss_robot import TrussRobot, Robot3D, Robot2D
import numpy as np
import scipy.optimize.minimize
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from dataclasses import dataclass


@dataclass
class MotionParams:
    objective: str = "Ldot"
    broken_rollers: list[int] = None
    dt: float = 0.001
    animate_robot: bool = True
    refresh_rate: int = 10
    verbose_print: bool = False

class MotionPlanner:
    def __init__(
        self,
        robot: TrussRobot,
        motion_params: MotionParams,
    ):
        self.robot = robot
        self.dim = self.robot.dim
        self.num_supports = len(self.robot.supports)
        self.target_node_idx = self.robot.target_node[0] - 1
        self.num_nodes = self.robot.num_nodes
        self.broken_rollers = motion_params.broken_rollers
        self.dt = motion_params.dt

        self.obj_str = motion_params.objective
        self.constraint_indices_range: dict[str, tuple[int]] = {}

        self.motion_viz: MotionViz = None

        self._calc_init_constraints()
        self._get_objective(self.obj_str)

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

    def _calc_init_constraints(self):
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

        constraints.append(
            {"type": "eq", "fun": lambda x: (self.Aeq @ x.T - self.beq.T).ravel()}
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

    def _calc_unit_vector(self, v):
        norm = np.linalg.norm(v)
        if norm == 0:
            return v
        return v / norm

    def _update_loopcon(self):
        self.A_loopcon = self.robot.triangle_loops @ self.robot.R
        start, stop = self.constraint_indices_range["loop"]
        self.Aeq[start:stop, :] = self.A_loopcon

    def _update_b_move(self, dir: np.ndarray):
        self.b_move = self._calc_unit_vector(dir.reshape(self.b_move.shape))

        start, stop = self.constraint_indices_range["move"]
        self.beq[start:stop, :] = self.b_move

    def move_ol(
        self, motion_params: MotionParams
    ) -> tuple[np.typing.NDArray, TrussRobot]:
        t = 0
        animate_robot = motion_params.animate_robot
        refresh_rate = motion_params.refresh_rate
        verbose_print = motion_params.verbose_print

        if animate_robot:
            self.motion_viz = MotionViz(self.robot, refresh_rate=refresh_rate)

        self.curr_goal_idx = 0

        for _ in range(len(self.robot.path.transformed_path)):
            if self.curr_goal_idx == 0:
                self.curr_goal_idx += 1
                continue  # Because the first point in the path is the starting position

            self.curr_goal = self.robot.path.transformed_path[self.curr_goal_idx]
            target_node_position = self.robot.get_target_nodes_pos()
            goal_direction = self.curr_goal - target_node_position

            self._update_b_move(goal_direction)
            count = 0

            while np.linalg.norm(goal_direction) > 0.01 and count < 10000:
                if self.motion_viz and count % refresh_rate == 0:
                    self.motion_viz.update_motion_coords(
                        target_node_position=target_node_position, b_move=self.b_move
                    )

                xd_opt, Ldot = self._get_opt_motion()

                self.robot.ol_update_and_store_positions_and_R(t, self.dt, xd_opt, Ldot)

                t += self.dt
                target_node_position = self.robot.get_target_nodes_pos()

                goal_direction = self.curr_goal - target_node_position
                self._update_b_move(goal_direction)
                self._update_loopcon()

                if verbose_print:
                    print(self.b_move)

                if self.motion_viz:
                    self.motion_viz.update_plot()

                count += 1

            self.curr_goal_idx += 1

        return self.robot.thetad_hist, self.robot

    def move_cl(
        self, animate_robot=True, refresh_rate=10, verbose_print=False
    ) -> tuple[np.typing.NDArray, TrussRobot]:
        t = 0

        if animate_robot:
            self.motion_viz = MotionViz(self.robot, refresh_rate=refresh_rate)

        self.curr_goal_idx = 0

        for _ in range(len(self.robot.path.transformed_path)):
            if self.curr_goal_idx == 0:
                self.curr_goal_idx += 1
                continue  # Because the first point in the path is the starting position

            self.curr_goal = self.robot.path.transformed_path[self.curr_goal_idx]

            target_node_position = self.robot.get_target_nodes_pos()

            goal_direction = self.curr_goal - target_node_position

            self._update_b_move(goal_direction)
            count = 0

            while np.linalg.norm(goal_direction) > 0.001 and count < 10000:
                if self.motion_viz and count % refresh_rate == 0:
                    self.motion_viz.update_motion_coords(
                        target_node_position=target_node_position, b_move=self.b_move
                    )

                xd_opt, Ldot = self._get_opt_motion()

                self.robot.cl_update_positions_and_R(t, self.dt, xd_opt, Ldot)

                t += self.dt
                target_node_position = self.robot.get_target_nodes_pos()

                goal_direction = self.curr_goal - target_node_position
                self._update_b_move(goal_direction)
                self._update_loopcon()

                if verbose_print:
                    print(self.b_move)

                if self.motion_viz:
                    self.motion_viz.update_plot()

                count += 1

            self.curr_goal_idx += 1


class MotionViz:
    def __init__(self, robot: TrussRobot, refresh_rate: int = 10):
        self.robot = robot
        self.fig, self.ax = self.robot.create_fig_ax()
        self.refresh_rate = refresh_rate
        self.count = 0
        self.init_animation()

    def init_animation(self):
        plt.ion()
        self.dot = self.robot.plot_dot(self.ax)
        self.quiver = None
        self.robot.plot_path(self.ax)
        self.robot.plot_robot(self.ax)

    def update_motion_coords(self, target_node_position, b_move):
        self.quiver = self.robot.update_arrow(
            self.ax, self.quiver, target_node_position, b_move
        )
        self.robot.update_dot(self.dot, target_node_position)

    def update_plot(self):
        if self.count % self.refresh_rate == 0:
            self.robot.update_plot()
        self.count += 1
        plt.pause(0.0001)


if __name__ == "__main__":
    # robot = Robot3D(1, ZYZrot=[90., -90.0, -90.0], path_scale=2, path_type="thin_byu", num_sides=8)
    robot = Robot2D(1, ZYZrot=[0], path_scale=1, path_type="polygon", num_sides=4)
    motion_params = MotionParams(animate_robot=True, refresh_rate=10, verbose_print=True, objective="Ldot", broken_rollers=None, dt=0.01)
    planner = MotionPlanner(robot=robot, motion_params=motion_params)

    thetads, robot = planner.move_ol(motion_params=motion_params)

