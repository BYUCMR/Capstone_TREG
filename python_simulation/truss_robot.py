import numpy as np
from scipy.linalg import block_diag

from abc import ABC, abstractmethod
from typing import Any, ClassVar, Generic, TypeVar

from linalg import Vector, Matrix, rot2D, rotx, roty, rotz
from truss_config import TrussConfig

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.quiver import Quiver
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

_AxesT = TypeVar('_AxesT', Axes, Axes3D)


class TrussRobot(ABC, Generic[_AxesT]):
    dim: ClassVar[int]

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
        self._generate_embedding_shape(config)

        self.path = path + self.positions[self.move_node[0]]

        self.pos_hist = [self.positions.copy()]
        self.xd_hist = [np.zeros(self.positions.shape)]
        self.Ldot_hist = [np.zeros((self.edges_nodes.shape[0], 1))]
        self.L_hist = [self._calc_edge_lengths(self.positions)]
        self.thetad_hist = [np.zeros((self.L2th.shape[0], 1))]
        self.theta_hist = [np.zeros((self.L2th.shape[0], 1))]
        self.t_hist = [0.]

        self._scatter = None
        self._lines = []
        self._labels = []
        self._fills = []

    def _generate_embedding_shape(self, config: TrussConfig) -> None:
        '''Generates the appropriate geometric information based on the user-specified embedding index'''
        self.supports = config.support_nodes - 1
        self.move_node = config.moving_nodes - 1
        self.positions = config.initial_pos.copy()
        self.edges_nodes  = config.edges - 1
        self.triangle_nodes  = config.triangles - 1

        self.num_nodes = len(self.positions)
        self.num_edges = len(self.edges_nodes)
        self.num_triangles = len(self.triangle_nodes)

        self._rotate_robot()
        triangle_incident_mat = np.array([[1, -1, 0],[0, 1, -1],[-1, 0, 1]])
        B_T = block_diag(*[triangle_incident_mat]*self.num_triangles)
        self.B_T = np.delete(B_T, [i for i in range(0, self.num_edges, 3)], axis=1)

        self.num_rollers = self.B_T.shape[1]

        self.L2th = np.linalg.pinv(self.B_T)
        self.rigidity = self._calc_rigidity_matrix(self.positions)
        self.adj = self._calc_adj_matrix()

        self.triangle_loops = block_diag(*[np.ones((1,3))]*self.num_triangles)
        self.support_indices = self._calc_support_indices()

    def _calc_edge_lengths(self, positions: Matrix) -> Matrix:
        lengths = np.zeros((self.edges_nodes.shape[0], 1))
        for i, (p1, p2) in enumerate(self.edges_nodes):
            length = np.linalg.norm(positions[p1, :] - positions[p2, :])
            lengths[i, 0] = length
        return lengths

    def _calc_rigidity_matrix(self, positions: Matrix) -> Matrix:
        R = np.zeros((self.num_edges, self.num_nodes * self.dim))  # Initialize the rigidity matrix

        for i in range(self.num_edges):
            # Extract the points
            x1 = positions[self.edges_nodes[i, 0], :]
            x2 = positions[self.edges_nodes[i, 1], :]
            norm_x1x2 = np.linalg.norm(x1 - x2)

            # Loop over the dimensions
            for j in range(self.dim):
                R[i, self.edges_nodes[i, 0] + self.num_nodes * j] = (x1[j] - x2[j]) / norm_x1x2
                R[i, self.edges_nodes[i, 1] + self.num_nodes * j] = -R[i, self.edges_nodes[i, 0] + self.num_nodes * j]

        return R

    def _calc_adj_matrix(self) -> Matrix:
        adj = np.zeros((self.num_nodes, self.num_nodes))
        adj[self.edges_nodes] = 1
        return adj + adj.T

    def ol_update_and_store_positions_and_rigidity(self, t: float, dt: float, xd: Matrix, Ldot: Matrix) -> None:
        self.t_hist.append(t+dt)
        xd = xd.reshape((-1, self.dim), order="F")
        self.positions += xd*dt

        self.pos_hist.append(self.positions.copy())
        self.xd_hist.append(xd.copy())
        self.Ldot_hist.append(Ldot.copy())
        self.thetad_hist.append(self.L2th @ Ldot)

        self.L_hist.append(self._calc_edge_lengths(self.positions))
        self.theta_hist.append(self.theta_hist[-1] + dt*self.thetad_hist[-1])

        self.rigidity = self._calc_rigidity_matrix(self.positions)

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

    def get_move_nodes_pos(self) -> Vector:
        return self.positions[self.move_node[0], :]

    def update_plot(self) -> None:
        if self._scatter is None or not self._lines or not self._labels:
            raise RuntimeError("plot_robot must be called before update_plot")

        coords_xyz = self.positions.T

        if self.dim == 2:
            x, y = coords_xyz[0], coords_xyz[1]
            z = np.zeros_like(y)
        if self.dim == 3:
            x, y, z = coords_xyz[0], coords_xyz[1], coords_xyz[2]

        # Update scatter points
        if self.dim == 2:
            self._scatter.set_offsets(np.c_[x, y])
        if self.dim == 3:
            self._scatter._offsets3d = (x, y, z)

        # Update lines
        for (p1, p2), line in zip(self.edges_nodes, self._lines):
            line.set_data([x[p1], x[p2]], [y[p1], y[p2]])
            if self.dim == 3:
                line.set_3d_properties([z[p1], z[p2]])

        # Update fills
        if self.dim == 2:
            for (p1, p2, p3), fill in zip(self.triangle_nodes, self._fills):
                fill.set_xy(np.array([[x[p1], y[p1]], [x[p2], y[p2]], [x[p3], y[p3]]]))

        # Update labels
        for label, xi, yi, zi in zip(self._labels, x, y, z):
            label.set_position((xi + 0.1, yi))
            if self.dim ==3:
                label.set_3d_properties(zi, zdir='z')

    def plot_dot(self, ax: _AxesT) -> Line2D:
        dot, = ax.plot(*[[] for _ in range(self.dim)], 'o', color="blue")
        dot.set_markerfacecolor('blue')  # fill color
        dot.set_markeredgecolor('gray')  # optional edge color
        dot.set_markersize(8)

        return dot

    def fk_position(self, t: float, dt: float, real_thetas: Matrix) -> None:
        real_thetas = real_thetas.reshape((-1, 1))
        average_theta_d = (real_thetas - self.theta_hist[-1]) / dt
        return self.fk_velocity(t, dt, average_theta_d)

    def fk_velocity(self, t: float, dt: float, real_thetads: Matrix) -> None:
        real_thetads = real_thetads.reshape((-1, 1))
        real_thetas = self.theta_hist[-1] + real_thetads*dt
        real_Ldot = self.B_T@real_thetads
        real_xd = self._convert_Ldot_to_xd(real_Ldot)
        real_xd = real_xd.reshape((-1, self.dim), order="F")
        self.positions = self.positions + real_xd*dt
        real_L = self._calc_edge_lengths(self.positions)
        self.rigidity = self._calc_rigidity_matrix(self.positions)
        self._cl_update_positions_and_rigidity(t, self.positions, real_xd, real_Ldot, real_thetads, real_L, real_thetas)

    def convert_xd_to_thetad(self, xd: Matrix) -> Matrix:
        xd = xd.reshape((-1, 1))
        Ldot = self.rigidity @ xd
        thetad = self.L2th@Ldot
        return thetad.reshape((1, -1))

    def _convert_Ldot_to_xd(self, Ldot: Matrix) -> Matrix:
        xd = np.zeros((self.num_nodes*self.dim, 1))
        rigidity_reduced = np.delete(self.rigidity, self.support_indices, axis=1)
        xd_reduced = np.linalg.inv(rigidity_reduced) @ Ldot

        value_position = [i for i in range(len(xd)) if i not in self.support_indices]
        xd[value_position, 0] = xd_reduced[:, 0]

        return xd

    def _calc_support_indices(self) -> list[np.int64]:
        support_indices: list[np.int64] = []
        for i, support in enumerate(self.supports):
            for d in range(i, self.dim):
                support_indices.append(support + d*self.num_nodes)
        support_indices.sort()
        return support_indices

    @abstractmethod
    def _rotate_robot(self) -> None:
        '''Rotates robot positions such that supports are on xy plane and origin is at first point'''
        raise NotImplementedError

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


class Robot2D(TrussRobot[Axes]):
    dim: ClassVar = 2

    def _rotate_robot(self) -> None:
        orign_node_idx = self.supports[0]
        linear_node_idx = self.supports[1]

        self.positions -= self.positions[orign_node_idx]
        theta = -np.arctan2(self.positions[linear_node_idx][1] - self.positions[orign_node_idx][1],
                            self.positions[linear_node_idx][0] - self.positions[orign_node_idx][0])

        self.positions = (rot2D(theta)@(self.positions.T)).T

    def plot_robot(self, ax: Axes) -> None:
        x, y = self.positions.T

        # Plot the points
        self._scatter = ax.scatter(x, y, color='b', s=50, label='Points')

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, (p1, p2) in enumerate(self.edges_nodes):
            line, = ax.plot([x[p1], x[p2]], [y[p1], y[p2]], triangle_colors[idx // 3], lw=3)
            self._lines.append(line)

        for idx, (p1, p2, p3) in enumerate(self.triangle_nodes):
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
        x, y = self.path.T
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
            x, y = self.positions.T
            xs.append(x)
            ys.append(y)

        if self.path_plotted:
            x, y = self.path.T
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

class Robot3D(TrussRobot[Axes3D]):
    dim: ClassVar = 2

    def _rotate_robot(self) -> None:
        '''Rotates robot positions such that supports are on xy plane and origin is at first point'''
        origin_node_idx = self.supports[0]  # This node should be centered at origin
        linear_node_idx = self.supports[1]  # This node should be aligned along the x axis
        free_node_idx = self.supports[2]  # This node should be on xy plane

        self.positions -= self.positions[origin_node_idx]
        theta_z = -np.arctan2(self.positions[linear_node_idx][1] - self.positions[origin_node_idx][1],
                              self.positions[linear_node_idx][0] - self.positions[origin_node_idx][0])
        self.positions = (rotz(theta_z)@(self.positions.T)).T
        theta_y = np.arctan2(self.positions[linear_node_idx, 2], self.positions[linear_node_idx, 0])
        self.positions = (roty(theta_y)@(self.positions.T)).T
        theta_x = -np.arctan2(self.positions[free_node_idx][2], self.positions[free_node_idx][1])
        self.positions = (rotx(theta_x)@(self.positions.T)).T
        # self.positions = clean_matrix(self.positions, tol=1e-12)

        if np.sum(self.positions[:, -1]) < 0:
            self.positions[:,-1] *= -1

    def plot_robot(self, ax: Axes3D) -> None:
        x, y, z = self.positions.T

        # Plot the points
        self._scatter = ax.scatter(x, y, z, color='b', s=50, label='Points')

        for line in self._lines:
            line.remove()
        self._lines.clear()

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, (p1, p2) in enumerate(self.edges_nodes):
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
        x, y, z = self.path.T
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
            x, y, z = self.positions.T
            xs.append(x)
            ys.append(y)
            zs.append(z)

        if self.path_plotted:
            x, y, z = self.path.T
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

    # robot = Robot2D(config_2d, path_2d)
    robot = Robot3D(config_3d, path_3d)

    fig, ax_robot, ax_theta = robot.create_fig_ax()
    robot.plot_robot(ax_robot)
    robot.plot_path(ax_robot)
    robot.show(ax_robot)
