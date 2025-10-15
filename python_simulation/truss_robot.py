import numpy as np
from scipy.linalg import block_diag

from abc import ABC, abstractmethod
from typing import Any, Generic, SupportsIndex, TypeVar

from linalg import Vector, Matrix, rot2D, rotx, roty, rotz
from truss_config import TrussConfig, IntMatrix, edges

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.quiver import Quiver
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

_AxesT = TypeVar('_AxesT', Axes, Axes3D)


def calc_rigidity_matrix(positions: Matrix, triangles: IntMatrix) -> Matrix:
    num_nodes, dim = positions.shape
    R = np.zeros((3*len(triangles), num_nodes*dim))
    for i, (e1, e2) in enumerate(edges(triangles)):
        x1 = positions[e1]
        x2 = positions[e2]
        dist = np.linalg.norm(x1 - x2)
        for j in range(dim):
            rel_dist_j = (x1[j] - x2[j]) / dist
            R[i, e1 + j*num_nodes] = rel_dist_j
            R[i, e2 + j*num_nodes] = -rel_dist_j
    return R


def calc_edge_lengths(positions: Matrix, triangles: IntMatrix) -> Matrix:
    lengths = np.zeros((3*len(triangles), 1))
    for i, (p1, p2) in enumerate(edges(triangles)):
        length = np.linalg.norm(positions[p1] - positions[p2])
        lengths[i, 0] = length
    return lengths


def rotate_2d(
    positions: Matrix,
    orign_node: SupportsIndex,
    linear_node: SupportsIndex,
) -> Matrix:
    positions = positions - positions[orign_node]
    theta = -np.arctan2(positions[linear_node, 1] - positions[orign_node, 1],
                        positions[linear_node, 0] - positions[orign_node, 0])
    return (rot2D(theta) @ positions.T).T


def rotate_3d(
    positions: Matrix,
    origin_node: SupportsIndex,
    linear_node: SupportsIndex,
    free_node: SupportsIndex,
) -> Matrix:
    """
    Rotate positions such that supports are on xy plane and the first point is the origin.

    `origin_node` will be set at the origin
    `linear_node` will be aligned along the x-axis
    `free_node` will be on the xy plane
    """
    positions = positions - positions[origin_node]
    theta_z = -np.arctan2(positions[linear_node, 1] - positions[origin_node, 1],
                          positions[linear_node, 0] - positions[origin_node, 0])
    positions = (rotz(theta_z) @ positions.T).T
    theta_y = np.arctan2(positions[linear_node, 2], positions[linear_node, 0])
    positions = (roty(theta_y) @ positions.T).T
    theta_x = -np.arctan2(positions[free_node, 2], positions[free_node, 1])
    positions = (rotx(theta_x) @ positions.T).T
    if np.sum(positions[:, -1]) < 0:
        positions[:,-1] *= -1
    return positions


class TrussRobot:
    def __init__(self, config: TrussConfig, path: Matrix) -> None:
        """
        Initializes the truss robot object with specified parameters.

        Args:
            config (TrussConfig): Configuration of the robot.
            path_type (str): Type of path for the robot to follow.
            RPYrot (tuple[float, ...]): Roll, pitch, and yaw rotation angles (in degrees).
            path_scale (float): Scaling factor for the path length.
            num_sides (int): Number of sides for the path shape.

        """
        self.config = config
        self.positions = config.initial_pos.copy()

        self.num_nodes, self.dim = self.positions.shape
        self.num_edges = 3 * len(self.config.triangles)
        self.num_triangles = len(self.config.triangles)

        self.positions = (rotate_2d if self.dim == 2 else rotate_3d)(self.positions, *self.config.supports)
        triangle_incident_mat = np.array([[1, -1, 0],[0, 1, -1],[-1, 0, 1]])
        B_T = block_diag(*[triangle_incident_mat]*self.num_triangles)
        self.B_T = np.delete(B_T, [i for i in range(0, self.num_edges, 3)], axis=1)

        self.num_rollers = self.B_T.shape[1]

        self.L2th = np.linalg.pinv(self.B_T)
        self.rigidity = calc_rigidity_matrix(self.positions, self.config.triangles)

        self.triangle_loops = block_diag(*[np.ones((1,3))]*self.num_triangles)
        self.support_indices = self._calc_support_indices()

        self.path = path + self.positions[self.config.move_node]

        self.pos_hist = [self.positions.copy()]
        self.xd_hist = [np.zeros(self.positions.shape)]
        self.Ldot_hist = [np.zeros((self.num_edges, 1))]
        self.L_hist = [calc_edge_lengths(self.positions, self.config.triangles)]
        self.thetad_hist = [np.zeros((self.L2th.shape[0], 1))]
        self.theta_hist = [np.zeros((self.L2th.shape[0], 1))]
        self.t_hist = [0.]

    @property
    def move_node_pos(self) -> Vector:
        return self.positions[self.config.move_node]

    def ol_update_and_store_positions_and_rigidity(self, t: float, dt: float, xd: Matrix, Ldot: Matrix) -> None:
        self.t_hist.append(t+dt)
        xd = xd.reshape((-1, self.dim), order="F")
        self.positions += xd*dt

        self.pos_hist.append(self.positions.copy())
        self.xd_hist.append(xd.copy())
        self.Ldot_hist.append(Ldot.copy())
        self.thetad_hist.append(self.L2th @ Ldot)

        self.L_hist.append(calc_edge_lengths(self.positions, self.config.triangles))
        self.theta_hist.append(self.theta_hist[-1] + dt*self.thetad_hist[-1])

        self.rigidity = calc_rigidity_matrix(self.positions, self.config.triangles)

    def _cl_update_positions_and_rigidity(
        self,
        t: float,
        positions: Matrix,
        xd: Matrix,
        Ldot: Matrix,
        thetad: Matrix,
        L: Matrix,
        theta: Matrix,
    ) -> None:
        self.t_hist.append(t)

        self.pos_hist.append(positions.copy())
        self.xd_hist.append(xd.copy())
        self.Ldot_hist.append(Ldot.copy())
        self.thetad_hist.append(thetad.copy())

        self.L_hist.append(L.copy())
        self.theta_hist.append(theta.copy())

    def fk_position(self, t: float, dt: float, real_thetas: Matrix) -> None:
        real_thetads = (real_thetas - self.theta_hist[-1]) / dt
        real_Ldot = self.B_T@real_thetads
        real_xd = self._convert_Ldot_to_xd(real_Ldot)
        real_xd = real_xd.reshape((-1, self.dim), order="F")
        self.positions = self.positions + real_xd*dt
        real_L = calc_edge_lengths(self.positions, self.config.triangles)
        self.rigidity = calc_rigidity_matrix(self.positions, self.config.triangles)
        self._cl_update_positions_and_rigidity(t, self.positions, real_xd, real_Ldot, real_thetads, real_L, real_thetas)

    def convert_xd_to_thetad(self, xd: Matrix) -> Matrix:
        return self.L2th @ self.rigidity @ xd

    def _convert_Ldot_to_xd(self, Ldot: Matrix) -> Matrix:
        xd = np.zeros((self.num_nodes*self.dim, 1))
        rigidity_reduced = np.delete(self.rigidity, self.support_indices, axis=1)
        xd_reduced = np.linalg.inv(rigidity_reduced) @ Ldot

        value_position = [i for i in range(len(xd)) if i not in self.support_indices]
        xd[value_position, 0] = xd_reduced[:, 0]

        return xd

    def _calc_support_indices(self) -> list[np.int64]:
        support_indices: list[np.int64] = []
        for i, support in enumerate(self.config.supports):
            for d in range(i, self.dim):
                support_indices.append(support + d*self.num_nodes)
        support_indices.sort()
        return support_indices


class RobotPlotter(ABC, Generic[_AxesT]):
    def __init__(self, robot: TrussRobot) -> None:
        self.robot = robot
        self._scatter = None
        self._lines = []
        self._labels = []
        self._fills = []

    def update_plot(self) -> None:
        if self._scatter is None or not self._lines or not self._labels:
            raise RuntimeError("plot_robot must be called before update_plot")

        coords_xyz = self.robot.positions.T

        if self.robot.dim == 2:
            x, y = coords_xyz[0], coords_xyz[1]
            z = np.zeros_like(y)
        if self.robot.dim == 3:
            x, y, z = coords_xyz[0], coords_xyz[1], coords_xyz[2]

        # Update scatter points
        if self.robot.dim == 2:
            self._scatter.set_offsets(np.c_[x, y])
        if self.robot.dim == 3:
            self._scatter._offsets3d = (x, y, z)

        # Update lines
        for (p1, p2), line in zip(edges(self.robot.config.triangles), self._lines):
            line.set_data([x[p1], x[p2]], [y[p1], y[p2]])
            if self.robot.dim == 3:
                line.set_3d_properties([z[p1], z[p2]])

        # Update fills
        if self.robot.dim == 2:
            for (p1, p2, p3), fill in zip(self.robot.config.triangles, self._fills):
                fill.set_xy(np.array([[x[p1], y[p1]], [x[p2], y[p2]], [x[p3], y[p3]]]))

        # Update labels
        for label, xi, yi, zi in zip(self._labels, x, y, z):
            label.set_position((xi + 0.1, yi))
            if self.robot.dim ==3:
                label.set_3d_properties(zi, zdir='z')

    def plot_dot(self, ax: _AxesT) -> Line2D:
        dot, = ax.plot(*[[] for _ in range(self.robot.dim)], 'o', color="blue")
        dot.set_markerfacecolor('blue')  # fill color
        dot.set_markeredgecolor('gray')  # optional edge color
        dot.set_markersize(8)

        return dot

    @abstractmethod
    def plot_robot(self, ax: _AxesT) -> None:
        raise NotImplementedError

    @abstractmethod
    def plot_path(self, ax: _AxesT, fill: bool = True) -> None:
        raise NotImplementedError

    @abstractmethod
    def create_fig_ax(self, equal_aspect: bool = True) -> tuple[Figure, _AxesT, Axes]:
        """Create a figure and return (fig, main_axis_for_robot, axis_for_theta_history)

        The extra axis is used by MotionViz to plot theta histories alongside the
        robot visualization.
        """
        raise NotImplementedError

    @abstractmethod
    def show(self, ax: _AxesT) -> None:
        raise NotImplementedError

    @staticmethod
    @abstractmethod
    def plot_arrow(ax: _AxesT, position: Vector, direction: Vector) -> Any:
        raise NotImplementedError

    @staticmethod
    @abstractmethod
    def update_dot(dot, position: Vector) -> None:
        raise NotImplementedError


class RobotPlotter2D(RobotPlotter[Axes]):
    def plot_robot(self, ax: Axes) -> None:
        x, y = self.robot.positions.T

        # Plot the points
        self._scatter = ax.scatter(x, y, color='b', s=50, label='Points')

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, (p1, p2) in enumerate(edges(self.robot.config.triangles)):
            line, = ax.plot([x[p1], x[p2]], [y[p1], y[p2]], triangle_colors[idx // 3], lw=3)
            self._lines.append(line)

        for idx, (p1, p2, p3) in enumerate(self.robot.config.triangles):
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

    def plot_path(self, ax: Axes, fill: bool = True) -> None:
        x, y = self.robot.path.T
        ax.plot(x, y)

        if fill:
            ax.fill(x, y, facecolor='red', alpha=0.5, hatch='--')

        self.path_plotted = True

    def create_fig_ax(self) -> tuple[Figure, Axes, Axes]:
        # Create a two-panel figure: left for robot, right for theta history
        fig, (ax_robot, ax_theta) = plt.subplots(ncols=2, figsize=(10, 5))
        return fig, ax_robot, ax_theta

    def show(self, ax: Axes) -> None:
        if not self.robot_plotted and not self.path_plotted:
            self.robot_plotted = False
            self.path_plotted = False
            plt.show()
            return

        xs, ys = [], []

        if self.robot_plotted:
            x, y = self.robot.positions.T
            xs.append(x)
            ys.append(y)

        if self.path_plotted:
            x, y = self.robot.path.T
            xs.append(x)
            ys.append(y)

        # Flatten all arrays into one for min/max computation
        all_x = np.concatenate(xs)
        all_y = np.concatenate(ys)

        min_all = min(np.min(all_x), np.min(all_y))
        max_all = max(np.max(all_x), np.max(all_y))

        # Set same limits
        ax.set_xlim(min_all - 1, max_all + 1)
        ax.set_ylim(min_all - 1, max_all + 1)

        # Set box aspect to equal
        ax.set_aspect('equal')

        self.robot_plotted = False
        self.path_plotted = False
        plt.show()

    @staticmethod
    def plot_arrow(ax: Axes, position: Vector, direction: Vector) -> Quiver:
        return ax.quiver(*position.tolist(), *direction.tolist(), color='blue', linewidth=3)

    @staticmethod
    def update_dot(dot, position) -> None:
        dot.set_data([position[0]], [position[1]])


class RobotPlotter3D(RobotPlotter[Axes3D]):
    def plot_robot(self, ax: Axes3D) -> None:
        x, y, z = self.robot.positions.T

        # Plot the points
        self._scatter = ax.scatter(x, y, z, color='b', s=50, label='Points')

        for line in self._lines:
            line.remove()
        self._lines.clear()

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, (p1, p2) in enumerate(edges(self.robot.config.triangles)):
            line, = ax.plot([x[p1], x[p2]], [y[p1], y[p2]], [z[p1], z[p2]], triangle_colors[idx // 3], lw=6)
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

    def plot_path(self, ax: Axes3D, fill: bool = True) -> None:
        x, y, z = self.robot.path.T
        ax.scatter(x, y, z, color='r')

        if fill:
            verts = [list(zip(x, y, z))]
            poly = Poly3DCollection(verts, alpha=0.5, facecolor='red')
            ax.add_collection3d(poly)

        self.path_plotted = True

    def create_fig_ax(self, equal_aspect: bool = True) -> tuple[Figure, Axes3D, Axes]:
        # Create a two-panel figure: left for robot (3D), right for theta history
        fig = plt.figure(figsize=(12, 6))
        ax_robot = fig.add_subplot(1, 2, 1, projection='3d')
        ax_theta = fig.add_subplot(1, 2, 2)
        if equal_aspect:
            ax_robot.set_box_aspect([1, 1, 1]) # type: ignore
        return fig, ax_robot, ax_theta

    def show(self, ax: Axes3D):
        if not self.robot_plotted and not self.path_plotted:
            self.robot_plotted = False
            self.path_plotted = False
            plt.show()
            return

        xs, ys, zs = [], [], []

        if self.robot_plotted:
            x, y, z = self.robot.positions.T
            xs.append(x)
            ys.append(y)
            zs.append(z)

        if self.path_plotted:
            x, y, z = self.robot.path.T
            xs.append(x)
            ys.append(y)
            zs.append(z)

        # Flatten all arrays into one for min/max computation
        all_x = np.concatenate(xs)
        all_y = np.concatenate(ys)
        all_z = np.concatenate(zs)

        min_all = min(np.min(all_x), np.min(all_y), np.min(all_z))
        max_all = max(np.max(all_x), np.max(all_y), np.max(all_z))

        # Set same limits
        ax.set_xlim(min_all - 1, max_all + 1)
        ax.set_ylim(min_all - 1, max_all + 1)

        # Set box aspect to equal
        ax.set_box_aspect([1, 1, 1])

        self.robot_plotted = False
        self.path_plotted = False
        plt.show()

    @staticmethod
    def plot_arrow(ax: Axes3D, position: Vector, direction: Vector) -> Line3DCollection:
        return ax.quiver(*position.tolist(), *direction.tolist(), color='blue', length=1, normalize=True, linewidth=6, arrow_length_ratio=0.4)

    @staticmethod
    def update_dot(dot, position: Vector) -> None:
        dot.set_data_3d([position[0]], [position[1]], [position[2]])


if __name__ == "__main__":
    import path, truss_config
    config_2d = truss_config.CONFIG_2D_1
    config_3d = truss_config.CONFIG_3D_1
    path_2d = path.make_path(dimension=2)
    path_3d = path.make_path(RPYrot=(45, 30, 45))

    robot = TrussRobot(config_3d, path_3d)
    robot_plotter = RobotPlotter3D(robot)

    fig, ax_robot, ax_theta = robot_plotter.create_fig_ax()
    robot_plotter.plot_robot(ax_robot)
    robot_plotter.plot_path(ax_robot)
    robot_plotter.show(ax_robot)
