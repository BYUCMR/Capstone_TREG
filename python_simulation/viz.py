# viz.py
import numpy as np

import matplotlib.pyplot as plt

from linalg import Matrix, Vector
from truss_robot import RobotPlotter2D, RobotPlotter3D, TrussRobot


class MotionViz:
    def __init__(self, robot: TrussRobot, refresh_rate: int = 10, theta_window: float = 2.0) -> None:
        """
        Initialize the visualization for the TrussRobot.
        Args:
            robot (TrussRobot): The robot instance to visualize.
            refresh_rate (int, optional): Number of frames between plot refreshes. Defaults to 10.
            theta_window (float, optional): Duration (in seconds) of theta data to display in the moving window. Defaults to 2.0.
        """
        self.robot = robot
        self.robot_plotter = RobotPlotter3D(robot) if robot.dim == 3 else RobotPlotter2D(robot)
        # create_fig_ax now returns (fig, ax_robot, ax_theta)
        self.fig, self.ax, self.ax_theta = self.robot_plotter.create_fig_ax()
        self.refresh_rate = refresh_rate
        self.count = 0
        # how many seconds of theta data to show (moving window)
        self.theta_window = float(theta_window)

    def init_animation(self, path: Matrix) -> None:
        plt.ion()
        self.dot = self.robot_plotter.plot_dot(self.ax)
        self.quiver = None
        self.robot_plotter.plot_path(path, self.ax)
        self.robot_plotter.plot_robot(self.ax)

        # Theta history plot (right subplot) and theta-dot subplot below it
        # Prepare history storage and one line per theta
        self.theta_lines = []
        self.theta_times = []
        self.theta_histories = []
        # histories for theta dot
        self.thetad_lines = []
        self.thetad_histories = []
        # create a second axis under ax_theta for theta dots
        pos = self.ax_theta.get_position()
        # split the available height: upper for theta, lower for thetad
        thetad_height = pos.height * 0.35
        theta_height = pos.height - thetad_height - 0.02
        # adjust theta axis to occupy the upper portion
        self.ax_theta.set_position([pos.x0, pos.y0 + thetad_height + 0.02, pos.width, theta_height])
        # add thetad axis below
        self.ax_thetad = self.fig.add_axes([pos.x0, pos.y0, pos.width, thetad_height])
        self.ax_thetad.set_xlabel('time')
        self.ax_thetad.set_ylabel('thetad')
        self.ax_thetad.grid(True)

        # visibility state per theta (default True)
        self.theta_visible = []
        # place to store thetad lines
        self.thetad_lines = []

        # number of theta values
        n_thetas = int(self.robot.roll.size)
        # initialize time and histories with initial theta
        t0 = float(self._get_robot_time())
        self.theta_times.append(t0)
        initial_theta = self.robot.roll.ravel()
        # try to get initial thetads; fall back to zeros
        try:
            initial_thetads = self.robot.rollrate.ravel()
        except Exception:
            initial_thetads = np.zeros_like(initial_theta)

        for i in range(n_thetas):
            self.theta_histories.append([float(initial_theta[i])])
            (line,) = self.ax_theta.plot([t0], [float(initial_theta[i])], label=f"θ{i+1}")
            self.theta_lines.append(line)
            # create corresponding thetad line and history
            (dline,) = self.ax_thetad.plot([t0], [float(initial_thetads[i])], label=f"θ{i+1}_dot", linestyle='--')
            self.thetad_lines.append(dline)
            self.thetad_histories.append([float(initial_thetads[i])])
            self.theta_visible.append(True)

        self.ax_theta.set_xlabel('time')
        self.ax_theta.set_ylabel('theta')
        self.ax_theta.grid(True)
        try:
            self.ax_theta.legend(loc='upper right')
        except Exception:
            pass

        try:
            from matplotlib.widgets import CheckButtons

            # compute positions so the control panel sits to the right of ax_theta
            theta_pos = self.ax_theta.get_position()
            # shrink theta axis a bit to make room
            new_theta_width = theta_pos.width * 0.78
            self.ax_theta.set_position([theta_pos.x0, theta_pos.y0, new_theta_width, theta_pos.height])
            control_x0 = theta_pos.x0 + new_theta_width + 0.02
            control_width = min(0.22, 1.0 - control_x0 - 0.01)
            if control_width <= 0:
                # fallback: place control inside the rightmost 20% if there's no room
                control_x0 = max(0.8, theta_pos.x0 + theta_pos.width + 0.01)
                control_width = 0.18

            control_ax = self.fig.add_axes([control_x0, theta_pos.y0, control_width, theta_pos.height])
            control_ax.axis('off')

            labels = [f"θ{i+1}" for i in range(n_thetas)]
            visibility = [True] * n_thetas
            # place the check buttons occupying the top portion of control_ax
            check_rect = [control_x0, theta_pos.y0 + theta_pos.height * 0.5, control_width, theta_pos.height * 0.45]
            check_ax = self.fig.add_axes(check_rect)
            self.check = CheckButtons(check_ax, labels, visibility)

            # The control panel will only contain the checkboxes. The checkboxes will toggle
            # visibility of both theta and thetad traces.
            def _toggle(label):
                idx = labels.index(label)
                self.theta_visible[idx] = not self.theta_visible[idx]
                self.theta_lines[idx].set_visible(self.theta_visible[idx])
                # also toggle the corresponding thetad line if present
                try:
                    self.thetad_lines[idx].set_visible(self.theta_visible[idx])
                except Exception:
                    pass
                # rescale to only the currently visible lines
                try:
                    self._rescale_visible_axes()
                except Exception:
                    plt.draw()

            self.check.on_clicked(_toggle)
        except Exception:
            # matplotlib might not have widgets in some backends; ignore
            self.check = None

    def _get_robot_time(self, default: float = 0.0) -> float:
        """Return a numeric timestamp from robot.t_hist whether it's a list or scalar."""
        if not hasattr(self.robot, 't_hist'):
            return default
        t = self.robot.t_hist[-1]
        try:
            # list-like with first element as time
            if isinstance(t, (list, tuple, np.ndarray)):
                if len(t) > 0:
                    return t[0]
                return default
            # scalar
            return float(t)
        except Exception:
            return default

    def update_motion_coords(self, move_node_position: Vector, b_move: Vector) -> None:
        if self.quiver is not None:
            self.quiver.remove()
        self.quiver = self.robot_plotter.plot_arrow(self.ax, move_node_position, b_move)
        self.robot_plotter.update_dot(self.dot, move_node_position)

    def _append_theta_data(self) -> None:
        # Append latest time and theta values
        t = float(self._get_robot_time(default=(self.theta_times[-1] if len(self.theta_times) else 0.0)))
        thetas = self.robot.roll.ravel()
        self.theta_times.append(t)
        for i, val in enumerate(thetas):
            self.theta_histories[i].append(float(val))

        # also append theta dot (thetad) values if available
        try:
            thetads = self.robot.rollrate.ravel()
        except Exception:
            thetads = [0.0] * len(thetas)

        for i, val in enumerate(thetads):
            # ensure history list exists
            if i >= len(self.thetad_histories):
                self.thetad_histories.append([float(val)])
            else:
                self.thetad_histories[i].append(float(val))

        # enforce moving window (trim old data)
        if len(self.theta_times) > 1 and self.theta_window > 0:
            t_now = self.theta_times[-1]
            t_cut = t_now - self.theta_window
            # find first index with time >= t_cut
            first_idx = 0
            for idx, tv in enumerate(self.theta_times):
                if tv >= t_cut:
                    first_idx = idx
                    break
            # trim
            if first_idx > 0:
                self.theta_times = self.theta_times[first_idx:]
                for i in range(len(self.theta_histories)):
                    self.theta_histories[i] = self.theta_histories[i][first_idx:]
                # trim thetad histories as well
                for i in range(len(self.thetad_histories)):
                    self.thetad_histories[i] = self.thetad_histories[i][first_idx:]

    def update_plot(self) -> None:
        if self.count % self.refresh_rate == 0:
            self.robot_plotter.update_plot()
        self.count += 1
        # update theta history plot
        try:
            self._append_theta_data()
            for i, line in enumerate(self.theta_lines):
                line.set_data(self.theta_times, self.theta_histories[i])
                line.set_visible(self.theta_visible[i])
            # update thetad lines if present
            for i, dline in enumerate(self.thetad_lines):
                if i < len(self.thetad_histories):
                    dline.set_data(self.theta_times, self.thetad_histories[i])
                dline.set_visible(self.theta_visible[i] if i < len(self.theta_visible) else True)

            # adjust x limits to show full history
            if len(self.theta_times) >= 2:
                # If using moving window, show only the last window seconds
                if self.theta_window > 0:
                    t_end = self.theta_times[-1]
                    t_start = max(self.theta_times[0], t_end - self.theta_window)
                    self.ax_theta.set_xlim(t_start, t_end)
                    # keep thetad axis in sync
                    try:
                        self.ax_thetad.set_xlim(t_start, t_end)
                    except Exception:
                        pass
                else:
                    self.ax_theta.set_xlim(self.theta_times[0], self.theta_times[-1])
                    try:
                        self.ax_thetad.set_xlim(self.theta_times[0], self.theta_times[-1])
                    except Exception:
                        pass

            # compute y limits from only visible theta histories
            vis_theta_vals = []
            for i, h in enumerate(self.theta_histories):
                if i < len(self.theta_visible) and self.theta_visible[i]:
                    vis_theta_vals.append(np.array(h))

            if len(vis_theta_vals) > 0:
                all_vals = np.concatenate(vis_theta_vals)
                vmin, vmax = float(np.min(all_vals)), float(np.max(all_vals))
                if vmin == vmax:
                    vmin -= 0.1
                    vmax += 0.1
                self.ax_theta.set_ylim(vmin - 0.1 * abs(vmin), vmax + 0.1 * abs(vmax))

            # compute y limits for thetad plot from only visible thetad histories
            vis_dvals = []
            for i, h in enumerate(self.thetad_histories):
                if i < len(self.theta_visible) and self.theta_visible[i]:
                    vis_dvals.append(np.array(h))

            if len(vis_dvals) > 0:
                all_dvals = np.concatenate(vis_dvals)
                dvmin, dvmax = float(np.min(all_dvals)), float(np.max(all_dvals))
                if dvmin == dvmax:
                    dvmin -= 0.1
                    dvmax += 0.1
                try:
                    self.ax_thetad.set_ylim(dvmin - 0.1 * abs(dvmin), dvmax + 0.1 * abs(dvmax))
                except Exception:
                    pass
        except Exception:
            # don't let plotting errors interrupt simulation
            pass

        plt.pause(0.001)

    def _rescale_visible_axes(self) -> None:
        """Rescale theta and thetad axes to show only visible traces.

        Uses the current theta_times for x-limits (respecting theta_window)
        and computes y-limits from the visible histories only.
        """
        # sync x-limits
        if len(self.theta_times) >= 1:
            if self.theta_window > 0:
                t_end = self.theta_times[-1]
                t_start = max(self.theta_times[0], t_end - self.theta_window)
            else:
                t_start, t_end = self.theta_times[0], self.theta_times[-1]
            try:
                self.ax_theta.set_xlim(t_start, t_end)
                self.ax_thetad.set_xlim(t_start, t_end)
            except Exception:
                pass

        # y-limits for theta using only visible traces
        vis_theta_vals = []
        for i, h in enumerate(self.theta_histories):
            if i < len(self.theta_visible) and self.theta_visible[i]:
                vis_theta_vals.append(np.array(h))
        if len(vis_theta_vals) > 0:
            all_vals = np.concatenate(vis_theta_vals)
            vmin, vmax = float(np.min(all_vals)), float(np.max(all_vals))
            if vmin == vmax:
                vmin -= 0.1
                vmax += 0.1
            self.ax_theta.set_ylim(vmin - 0.1 * abs(vmin), vmax + 0.1 * abs(vmax))

        # y-limits for thetad using only visible traces
        vis_dvals = []
        for i, h in enumerate(self.thetad_histories):
            if i < len(self.theta_visible) and self.theta_visible[i]:
                vis_dvals.append(np.array(h))
        if len(vis_dvals) > 0:
            all_dvals = np.concatenate(vis_dvals)
            dvmin, dvmax = float(np.min(all_dvals)), float(np.max(all_dvals))
            if dvmin == dvmax:
                dvmin -= 0.1
                dvmax += 0.1
            try:
                self.ax_thetad.set_ylim(dvmin - 0.1 * abs(dvmin), dvmax + 0.1 * abs(dvmax))
            except Exception:
                pass
        plt.draw()


def plot_theta_thetad(robot: TrussRobot, save_fig: bool = False, filename: str = "theta_thetad_plots.png") -> None:
    plt.ioff()
    fig, (ax_theta, ax_thetad) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    theta_hist = np.array(robot.roll_hist)
    thetad_hist = np.array(robot.rollrate_hist)

    # Plot each joint's theta
    for i in range(theta_hist.shape[1]):
        ax_theta.plot(robot.t_hist, theta_hist[:, i], label=f"theta {i+1}")
    ax_theta.set_ylabel('Theta')
    ax_theta.legend()
    ax_theta.set_title('Theta over Time')
    ax_theta.grid(True)

    # Plot each joint's thetad
    for i in range(thetad_hist.shape[1]):
        ax_thetad.plot(robot.t_hist, thetad_hist[:, i], label=f"thetad {i+1}")
    ax_thetad.set_xlabel('Time (s)')
    ax_thetad.set_ylabel('Thetad')
    ax_thetad.legend()
    ax_thetad.set_title('Thetad over Time')
    ax_thetad.grid(True)

    plt.tight_layout()
    if save_fig:
        fig.savefig(filename, dpi=150)
    plt.show()
