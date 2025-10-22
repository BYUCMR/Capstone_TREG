import numpy as np
from collections.abc import Iterable
from typing import SupportsIndex

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from linalg import Matrix, Vector
from truss_robot import RobotPlotter, RobotPlotter2D, RobotPlotter3D, TrussRobot


class MotionVis:
    def __init__(self, plotter: RobotPlotter, *, axes: Axes, refresh_rate: int = 10) -> None:
        self.robot_plotter = plotter
        self.axes = axes
        self.quiver = None
        self.refresh_rate = refresh_rate
        self.count = 0

    def init_animation(self, path: Matrix) -> None:
        self.dot = self.robot_plotter.plot_dot(self.axes)
        self.robot_plotter.plot_path(path, self.axes)
        self.robot_plotter.plot_robot(self.axes)

    def update_motion_coords(self, move_node_position: Vector, b_move: Vector) -> None:
        if self.count % self.refresh_rate:
            return
        if self.quiver is not None:
            self.quiver.remove()
        self.quiver = self.robot_plotter.plot_arrow(self.axes, move_node_position, b_move)
        self.robot_plotter.update_dot(self.dot, move_node_position)

    def update_plot(self) -> None:
        if self.count % self.refresh_rate == 0:
            self.robot_plotter.update_plot()
        self.count += 1


class MotionPlot:
    def __init__(
        self,
        *,
        robot: TrussRobot,
        fig: Figure,
        axes: Axes,
        shown_rollers: Iterable[SupportsIndex] | None = None,
        window_size: int = 200,
    ) -> None:
        self.robot = robot
        self.fig = fig
        self.axes = axes
        self.fig, self.axes = fig, axes
        self.last = 1
        self.window_size = window_size
        if shown_rollers is None:
            self.shown_rollers = np.arange(robot.num_rollers)
        else:
            self.shown_rollers = np.array(shown_rollers)
        self.theta_lines = []
        self.thetad_lines = []

    def get_data(self) -> tuple[Vector, Matrix, Matrix]:
        window = slice(max(0, self.last-self.window_size), self.last)
        t = np.array(self.robot.t_hist[window])
        roll = np.hstack(self.robot.roll_hist[window])[self.shown_rollers]
        rollrate = np.hstack(self.robot.rollrate_hist[window])[self.shown_rollers]
        return t, roll, rollrate

    def init_animation(self) -> None:
        # Theta history plot (right subplot) and theta-dot subplot below it
        # create a second axis under ax_theta for theta dots
        pos = self.axes.get_position()
        # split the available height: upper for theta, lower for thetad
        thetad_height = pos.height * 0.35
        theta_height = pos.height - thetad_height - 0.02
        # adjust theta axis to occupy the upper portion
        self.axes.set_position((pos.x0, pos.y0 + thetad_height + 0.02, pos.width, theta_height))
        # add thetad axis below
        self.ax_thetad = self.fig.add_axes((pos.x0, pos.y0, pos.width, thetad_height))
        self.ax_thetad.set_xlabel('time')
        self.ax_thetad.set_ylabel('thetad')
        self.ax_thetad.grid(True)

        t, roll, rollrate = self.get_data()
        for i in range(len(roll)):
            line, = self.axes.plot(t, roll[i], label=f"θ{i+1}")
            dline, = self.ax_thetad.plot(t, rollrate[i], label=f"θ{i+1}_dot", linestyle='--')
            self.theta_lines.append(line)
            self.thetad_lines.append(dline)

        self.axes.set_xlabel('time')
        self.axes.set_ylabel('theta')
        self.axes.grid(True)
        self.axes.legend(loc='upper right')

    def update_plot(self) -> None:
        self.last += 1
        t, roll, rollrate = self.get_data()
        for line, data in zip(self.theta_lines, roll, strict=True):
            line.set_data(t, data)
        for line, data in zip(self.thetad_lines, rollrate, strict=True):
            line.set_data(t, data)

        if len(t) > 1:
            self.axes.set_xlim(t[0], t[-1])
            self.ax_thetad.set_xlim(t[0], t[-1])
        if roll.size > 0:
            vmin, vmax = float(np.min(roll)), float(np.max(roll))
            self.axes.set_ylim(vmin-0.1, vmax+0.1)
        if rollrate.size > 0:
            dvmin, dvmax = float(np.min(rollrate)), float(np.max(rollrate))
            self.ax_thetad.set_ylim(dvmin-0.1, dvmax+0.1)


class MotionFig:
    def __init__(
        self,
        robot: TrussRobot,
        *,
        shown_rollers: Iterable[SupportsIndex] | None = None,
        refresh_rate: int = 10,
        window_size: int = 200,
    ) -> None:
        plotter = RobotPlotter3D(robot) if robot.dim == 3 else RobotPlotter2D(robot)
        fig, ax, ax_theta = plotter.create_fig_ax()
        self.motion_vis = MotionVis(plotter, axes=ax, refresh_rate=refresh_rate)
        self.motion_plot = MotionPlot(
            robot=robot,
            fig=fig,
            axes=ax_theta,
            shown_rollers=shown_rollers,
            window_size=window_size,
        )

    def init_animation(self, path: Matrix) -> None:
        plt.ion()
        self.motion_vis.init_animation(path)
        self.motion_plot.init_animation()

    def update_motion_coords(self, move_node_position: Vector, b_move: Vector) -> None:
        self.motion_vis.update_motion_coords(move_node_position, b_move)

    def update_plot(self) -> None:
        self.motion_vis.update_plot()
        self.motion_plot.update_plot()
        plt.pause(0.001)


def plot_theta_thetad(robot: TrussRobot, *, save_fig: bool = False, filename: str = "theta_thetad_plots.png") -> None:
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
