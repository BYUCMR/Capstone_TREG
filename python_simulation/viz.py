import numpy as np
from collections.abc import Generator, Iterable
from typing import SupportsIndex

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from linalg import Matrix, Vector
from truss_robot import RobotPlotter, RobotPlotter2D, RobotPlotter3D, TrussRobot


def display_robot(
    plotter: RobotPlotter, path: Matrix, *, axes: Axes, refresh_rate: int = 10
) -> Generator[None, tuple[Vector, Vector], None]:
    quiver = None
    count = -1
    dot = plotter.plot_dot(axes)
    plotter.plot_path(path, axes)
    plotter.plot_robot(axes)
    while True:
        move_node_pos, move_node_vel = yield
        count += 1
        if count % refresh_rate:
            continue
        plotter.update_plot()
        plotter.update_dot(dot, move_node_pos)
        if quiver is not None:
            quiver.remove()
        quiver = plotter.plot_arrow(axes, move_node_pos, move_node_vel)


def plot_roll(
    robot: TrussRobot,
    *,
    fig: Figure,
    axes: Axes,
    shown_rollers: Iterable[SupportsIndex] | None = None,
    window_size: int = 200,
) -> Generator[None]:
    last = 1
    if shown_rollers is None:
        shown_rollers = np.arange(robot.num_rollers)
    else:
        shown_rollers = np.array(shown_rollers)

    def get_data() -> tuple[Vector, Matrix, Matrix]:
        window = slice(max(0, last-window_size), last)
        t = np.array(robot.t_hist[window])
        roll = np.hstack(robot.roll_hist[window])[shown_rollers]
        rollrate = np.hstack(robot.rollrate_hist[window])[shown_rollers]
        return t, roll, rollrate

    # Theta history plot (right subplot) and theta-dot subplot below it
    # create a second axis under ax_theta for theta dots
    pos = axes.get_position()
    # split the available height: upper for theta, lower for thetad
    thetad_height = pos.height * 0.35
    theta_height = pos.height - thetad_height - 0.02
    # adjust theta axis to occupy the upper portion
    axes.set_position((pos.x0, pos.y0 + thetad_height + 0.02, pos.width, theta_height))
    # add thetad axis below
    ax_thetad = fig.add_axes((pos.x0, pos.y0, pos.width, thetad_height))
    ax_thetad.set_xlabel('time')
    ax_thetad.set_ylabel('thetad')
    ax_thetad.grid(True)

    theta_lines = []
    thetad_lines = []
    t, roll, rollrate = get_data()
    for i in range(len(roll)):
        line, = axes.plot(t, roll[i], label=f"θ{i+1}")
        dline, = ax_thetad.plot(t, rollrate[i], label=f"θ{i+1}_dot", linestyle='--')
        theta_lines.append(line)
        thetad_lines.append(dline)

    axes.set_xlabel('time')
    axes.set_ylabel('theta')
    axes.grid(True)
    axes.legend(loc='upper right')

    while True:
        yield
        last += 1
        t, roll, rollrate = get_data()

        for line, data in zip(theta_lines, roll, strict=True):
            line.set_data(t, data)
        for line, data in zip(thetad_lines, rollrate, strict=True):
            line.set_data(t, data)

        if len(t) > 1:
            axes.set_xlim(t[0], t[-1])
            ax_thetad.set_xlim(t[0], t[-1])
        if roll.size > 0:
            vmin, vmax = float(np.min(roll)), float(np.max(roll))
            axes.set_ylim(vmin-0.1, vmax+0.1)
        if rollrate.size > 0:
            dvmin, dvmax = float(np.min(rollrate)), float(np.max(rollrate))
            ax_thetad.set_ylim(dvmin-0.1, dvmax+0.1)


def make_motion_fig(
    robot: TrussRobot,
    path: Matrix,
    *,
    shown_rollers: Iterable[SupportsIndex] | None = None,
    refresh_rate: int = 10,
    window_size: int = 200,
) -> Generator[None, tuple[Vector, Vector], None]:
    plotter = RobotPlotter3D(robot) if robot.dim == 3 else RobotPlotter2D(robot)
    fig, ax, ax_theta = plotter.create_fig_ax()
    robot_display = display_robot(plotter, path, axes=ax, refresh_rate=refresh_rate)
    roll_plot = plot_roll(
        robot,
        fig=fig,
        axes=ax_theta,
        shown_rollers=shown_rollers,
        window_size=window_size,
    )
    plt.ion()
    next(robot_display)
    next(roll_plot)
    while True:
        move_node_pos, move_node_vel = yield
        robot_display.send((move_node_pos, move_node_vel))
        next(roll_plot)
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
