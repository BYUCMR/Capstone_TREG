import numpy as np
from collections.abc import Generator
from typing import Any, Protocol, TypeVar

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.quiver import Quiver
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
from mpl_toolkits.mplot3d.axes3d import Axes3D

from .gentools import auto_initialize
from .linalg import Matrix, Vector
from .robot import Robot
import plotly.graph_objects as go

_AxesT = TypeVar('_AxesT', Axes, Axes3D)


class RobotPlotter(Protocol[_AxesT]):
    def update_plot(self) -> None: ...
    @staticmethod
    def plot_dot(ax: _AxesT) -> Line2D: ...
    def plot_robot(self, ax: _AxesT) -> None: ...
    def plot_path(self, path: Matrix, ax: _AxesT, *, fill: bool = True) -> None: ...
    @staticmethod
    def create_fig_ax() -> tuple[Figure, _AxesT]: ...
    @staticmethod
    def create_fig_ax_theta() -> tuple[Figure, _AxesT, Axes]: ...
    def show(self, path: Matrix, ax: _AxesT) -> None: ...
    @staticmethod
    def plot_arrow(ax: _AxesT, position: Vector, direction: Vector) -> Any: ...
    @staticmethod
    def update_dot(dot, position: Vector) -> None: ...


class RobotPlotter2D(RobotPlotter[Axes]):
    def __init__(self, robot: Robot) -> None:
        self.robot = robot
        self._scatter = None
        self._lines = []
        self._labels = []
        self._fills = []

    def update_plot(self) -> None:
        if self._scatter is None or not self._lines or not self._labels:
            raise RuntimeError("plot_robot must be called before update_plot")

        x, y = self.robot.pos.T
        z = np.zeros_like(y)

        # Update scatter points
        self._scatter.set_offsets(np.c_[x, y])

        # Update lines
        for (p1, p2), line in zip(self.robot.config.triangles.links, self._lines):
            line.set_data([x[p1], x[p2]], [y[p1], y[p2]])

        # Update fills
        for tri, fill in zip(self.robot.config.triangles, self._fills):
            p1, p2, p3, _ = tri.nodes
            fill.set_xy(np.array([[x[p1], y[p1]], [x[p2], y[p2]], [x[p3], y[p3]]]))

        # Update labels
        for label, xi, yi, zi in zip(self._labels, x, y, z):
            label.set_position((xi + 0.1, yi))

    @staticmethod
    def plot_dot(ax: Axes) -> Line2D:
        dot, = ax.plot([], [], 'o', color="blue")
        dot.set_markerfacecolor('blue')  # fill color
        dot.set_markeredgecolor('gray')  # optional edge color
        dot.set_markersize(8)
        return dot

    def plot_robot(self, ax: Axes) -> None:
        x, y = self.robot.pos.T

        # Plot the points
        self._scatter = ax.scatter(x, y, color='b', s=50, label='Points')

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, (p1, p2) in enumerate(self.robot.config.triangles.links):
            line, = ax.plot([x[p1], x[p2]], [y[p1], y[p2]], triangle_colors[idx // 3], lw=3)
            self._lines.append(line)

        for idx, tri in enumerate(self.robot.config.triangles):
            p1, p2, p3, _ = tri.nodes
            self._fills.append(ax.fill([x[p1], x[p2], x[p3]], [y[p1], y[p2], y[p3]], triangle_colors[idx], alpha=0.2)[0])

        for i, (xi, yi) in enumerate(zip(x, y)):
            label = ax.text(xi + 0.1, yi, str(i + 1), fontsize=12)
            self._labels.append(label)

        # Add labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        # Add grid and set aspect ratio
        ax.grid(True)
        ax.set_aspect('equal') # This ensures equal aspect ratio

        # Add legend
        ax.legend()

        self.robot_plotted = True

    def plot_path(self, path: Matrix, ax: Axes, *, fill: bool = True) -> None:
        x, y = path.T
        ax.plot(x, y)
        if fill:
            ax.fill(x, y, facecolor='red', alpha=0.5, hatch='--')
        self.path_plotted = True

    @staticmethod
    def create_fig_ax() -> tuple[Figure, Axes]:
        return plt.subplots(ncols=1, figsize=(10, 5))

    @staticmethod
    def create_fig_ax_theta() -> tuple[Figure, Axes, Axes]:
        # Create a two-panel figure: left for robot, right for theta history
        fig, (ax_robot, ax_theta) = plt.subplots(ncols=2, figsize=(10, 5))
        return fig, ax_robot, ax_theta

    def show(self, path: Matrix, ax: Axes) -> None:
        if self.robot_plotted and self.path_plotted:
            points = np.vstack([self.robot.pos, path])
        elif self.robot_plotted and not self.path_plotted:
            points = self.robot.pos
        elif self.path_plotted:
            points = path
        else:
            plt.show()
            return
        lower = float(np.min(points)) - 1
        upper = float(np.max(points)) + 1

        ax.set_xlim(lower, upper)
        ax.set_ylim(lower, upper)
        ax.set_aspect('equal')

        self.robot_plotted = False
        self.path_plotted = False
        plt.show()

    @staticmethod
    def plot_arrow(ax: Axes, position: Vector, direction: Vector) -> Quiver:
        return ax.quiver(*position, *direction, color='blue', linewidth=3)

    @staticmethod
    def update_dot(dot: Line2D, position: Vector) -> None:
        dot.set_data(position.reshape(2, 1))


class RobotPlotter3D(RobotPlotter[Axes3D]):
    def __init__(self, robot: Robot) -> None:
        self.robot = robot
        self._scatter = None
        self._lines = []
        self._labels = []
        self._fills = []

    def update_plot(self) -> None:
        if self._scatter is None or not self._lines or not self._labels:
            raise RuntimeError("plot_robot must be called before update_plot")

        x, y, z = self.robot.pos.T

        # Update scatter points
        self._scatter._offsets3d = (x, y, z)

        # Update lines
        for (p1, p2), line in zip(self.robot.config.triangles.links, self._lines):
            line.set_data([x[p1], x[p2]], [y[p1], y[p2]])
            line.set_3d_properties([z[p1], z[p2]])

        # Update labels
        for label, xi, yi, zi in zip(self._labels, x, y, z):
            label.set_position((xi + 0.1, yi))
            label.set_3d_properties(zi, zdir='z')

    @staticmethod
    def plot_dot(ax: Axes3D) -> Line2D:
        dot, = ax.plot([], [], [], 'o', color="blue")
        dot.set_markerfacecolor('blue')  # fill color
        dot.set_markeredgecolor('gray')  # optional edge color
        dot.set_markersize(8)
        return dot

    def plot_robot(self, ax: Axes3D) -> None:
        x, y, z = self.robot.pos.T

        # Plot the points
        self._scatter = ax.scatter(x, y, z, color='b', s=50, label='Points')

        for line in self._lines:
            line.remove()
        self._lines.clear()

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for i, triangle in enumerate(self.robot.config.triangles):
            color = triangle_colors[i]
            for p1, p2 in triangle.links:
                line, = ax.plot([x[p1], x[p2]], [y[p1], y[p2]], [z[p1], z[p2]], color, lw=6)
                self._lines.append(line)

        for t in self._labels:
            t.remove()
        self._labels.clear()

        for i, (xi, yi, zi) in enumerate(zip(x, y, z)):
            label = ax.text(xi + 0.1, yi, zi, str(i + 1), fontsize=12)
            self._labels.append(label)

        # Add labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Add grid and set aspect ratio
        ax.grid(True)
        ax.set_box_aspect([1, 1, 1])  # This ensures equal aspect ratio

        # Add legend
        ax.legend()

        self.robot_plotted = True

    def plot_path(self, path: Matrix, ax: Axes3D, *, fill: bool = True) -> None:
        x, y, z = path.T
        ax.scatter(x, y, z, color='r')
        if fill:
            poly = Poly3DCollection([path], alpha=0.5, facecolor='red')
            ax.add_collection3d(poly)
        self.path_plotted = True

    @staticmethod
    def create_fig_ax() -> tuple[Figure, Axes3D]:
        fig = plt.figure(figsize=(12, 6))
        ax_robot = fig.add_subplot(projection='3d')
        ax_robot.set_box_aspect([1, 1, 1])
        return fig, ax_robot

    @staticmethod
    def create_fig_ax_theta() -> tuple[Figure, Axes3D, Axes]:
        # Create a two-panel figure: left for robot (3D), right for theta history
        fig = plt.figure(figsize=(12, 6))
        ax_robot = fig.add_subplot(1, 2, 1, projection='3d')
        ax_theta = fig.add_subplot(1, 2, 2)
        ax_robot.set_box_aspect([1, 1, 1])
        return fig, ax_robot, ax_theta

    def show(self, path: Matrix, ax: Axes3D):
        if self.robot_plotted and self.path_plotted:
            points = np.vstack([self.robot.pos, path])
        elif self.robot_plotted and not self.path_plotted:
            points = self.robot.pos
        elif self.path_plotted:
            points = path
        else:
            plt.show()
            return
        lower = float(np.min(points)) - 1
        upper = float(np.max(points)) + 1

        ax.set_xlim(lower, upper)
        ax.set_ylim(lower, upper)
        ax.set_box_aspect([1, 1, 1])

        self.robot_plotted = False
        self.path_plotted = False
        plt.show()

    @staticmethod
    def plot_arrow(ax: Axes3D, position: Vector, direction: Vector) -> Line3DCollection:
        return ax.quiver(*position, *direction, color='blue', length=1, normalize=True, linewidth=6, arrow_length_ratio=0.4)

    @staticmethod
    def update_dot(dot, position: Vector) -> None:
        dot.set_data_3d(position.reshape(3, 1))


class RoverPlotter3D:
    def __init__(self, robot: Robot) -> None:
        self.robot = robot

    def plot_payload(self) -> tuple[list[go.Scatter3d], go.Mesh3d]:
        payload = self.robot.config.payload
        payload_ind = set[int]()
        for i in payload:
            payload_ind.update(i.nodes)

        payload_pnts = self.robot.pos[list(payload_ind)]
        x = payload_pnts[:, 0]  # First column
        y = payload_pnts[:, 1]  # Second column
        z = payload_pnts[:, 2]
        payload_faces = [[0, 1, 2], [3, 4, 5],
                         [0, 3, 5], [0, 2, 5],
                         [1, 4, 5], [1, 2, 5],
                         [0, 1, 4], [0, 3, 4]]

        index_map = {v: i for i, v in enumerate(payload_ind)}

        # Replace each value in the pairs with its corresponding index
        payload_edges = [(index_map[a], index_map[b]) for a, b in payload.links]

        edge_lines: list[go.Scatter3d] = []
        for e in payload_edges:
            edge_lines.append(
                go.Scatter3d(
                    x=[x[e[0]], x[e[1]]],
                    y=[y[e[0]], y[e[1]]],
                    z=[z[e[0]], z[e[1]]],
                    mode='lines',
                    line=dict(color='black', width=4),
                    showlegend=False
                )
            )

        mesh = go.Mesh3d(
            x=x, y=y, z=z,
            i=[f[0] for f in payload_faces],
            j=[f[1] for f in payload_faces],
            k=[f[2] for f in payload_faces],
            color='lightblue',
            opacity=1,
            flatshading=True,
            name='Prism'
        )
        return edge_lines, mesh

    def plot_triangles(self) -> list[go.Scatter3d]:
        triangle_colors = {0: 'blue', 1: 'red', 2: 'orange', 3: 'green', 4: 'brown', 5: 'yellow', 6: 'purple'}
        traces: list[go.Scatter3d] = []
        for t, tri in enumerate(self.robot.config.triangles):
            v0, v1, v2, _ = tri.nodes
            x, y, z = ([self.robot.pos[v][i] for v in (v0, v1, v2, v0)] for i in range(3))
            trace = go.Scatter3d(
                x=x, y=y, z=z,
                mode='lines',
                line=dict(color=triangle_colors.get(t, 'gray'), width=6),
                marker=dict(size=5, color=triangle_colors.get(t, 'gray')),
                showlegend=False
            )
            traces.append(trace)
        return traces

    def plot_path(self, r_path: Matrix) -> go.Scatter3d:
        x, y, z = r_path.T
        return go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines+markers',
            line=dict(color='black', width=6),
            marker=dict(size=5, color='black', symbol='circle'),
            name='Path'
        )

    def generate_data(self, path: Matrix) -> list[go.Mesh3d | go.Scatter3d]:
        payload_scatter, payload_mesh = self.plot_payload()
        triangle_scatter = self.plot_triangles()
        path_scatter = self.plot_path(path)
        return [payload_mesh, path_scatter, *payload_scatter, *triangle_scatter]


@auto_initialize
def display_robot(
    plotter: RobotPlotter[_AxesT], *, path: Matrix, axes: _AxesT, refresh_rate: int = 10
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
        # if quiver is not None:
        #     quiver.remove()
        # quiver = plotter.plot_arrow(axes, move_node_pos, move_node_vel)


@auto_initialize
def animate_robot(
    plotter: RobotPlotter,
    path: Matrix,
    *,
    refresh_rate: int = 5,
) -> Generator[None, tuple[Vector, Vector], None]:
    fig, ax = plotter.create_fig_ax()
    robot_display = display_robot(plotter, path=path, axes=ax, refresh_rate=refresh_rate)
    plt.ion()
    while True:
        move_node_pos, move_node_vel = yield
        robot_display.send((move_node_pos, move_node_vel))
        plt.pause(0.001)
