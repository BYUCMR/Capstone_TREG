import numpy as np
from collections.abc import Generator, Iterable
from typing import SupportsIndex

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

import anim
from gentools import auto_initialize
from linalg import Matrix, Vector
from robot import Robot


@auto_initialize
def plot_roll(
    robot: Robot,
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


@auto_initialize
def make_motion_fig(
    robot: Robot,
    path: Matrix,
    *,
    shown_rollers: Iterable[SupportsIndex] | None = None,
    refresh_rate: int = 10,
    window_size: int = 200,
) -> Generator[None, tuple[Vector, Vector], None]:
    plotter = anim.RobotPlotter3D(robot) if robot.dim == 3 else anim.RobotPlotter2D(robot)
    fig, ax, ax_theta = plotter.create_fig_ax()
    robot_display = anim.display_robot(plotter, path=path, axes=ax, refresh_rate=refresh_rate)
    roll_plot = plot_roll(
        robot,
        fig=fig,
        axes=ax_theta,
        shown_rollers=shown_rollers,
        window_size=window_size,
    )
    plt.ion()
    while True:
        move_node_pos, move_node_vel = yield
        robot_display.send((move_node_pos, move_node_vel))
        next(roll_plot)
        plt.pause(0.001)


def plot_theta_thetad(robot: Robot, *, save_fig: bool = False, filename: str = "theta_thetad_plots.png") -> None:
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


if __name__ == "__main__":
    import path, truss_config
    from robot import Robot
    config_3d = truss_config.CONFIG_3D_1
    config_2d = truss_config.CONFIG_2D_1
    rover = truss_config.CONFIG_3D_ROVER1
    path_3d = path.make_path(RPYrot=(90., -45.0, 45.0))
    path_2d = path.make_path(dimension=2)

    ol_robot = Robot(config_2d)
    path_2d += ol_robot.move_node_pos
    fig = make_motion_fig(ol_robot, path_2d)
    for pos, vel in ol_robot.move_node_along_path(ol_robot.config.move_node, path_2d):
        fig.send((pos, vel))
