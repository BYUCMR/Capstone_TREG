import numpy as np
from common_data import Data3D, Data2D
from scipy.linalg import block_diag
from path import Path

from abc import ABC, abstractmethod

from util import rot2D, rotx, roty, rotz

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib.figure import Figure
from matplotlib.quiver import Quiver
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class TrussRobot(ABC):

    def __init__(self, index: int, dimension: int, path_type: str, RPYrot: list[float], path_scale: float, num_sides: int):
        """
        Initializes the truss robot object with specified parameters.

        Args:
            index (int): Index of the robot instance.
            dimension (int): Dimensionality of the robot (2 for 2D, 3 for 3D).
            path_type (str): Type of path for the robot to follow.
            RPYrot (list[float]): Roll, pitch, and yaw rotation angles (in radians or degrees).
            path_scale (float): Scaling factor for the path length.
            num_sides (int): Number of sides for the path shape.

        """

        self.index = index
        self.dim = dimension
        if self.dim == 3:
            self.data = Data3D()
        else:
            self.data = Data2D()
        self._generate_embedding_shape(self.index)

        self.path = Path(path_type,
                         self.dim,
                         length=path_scale,
                         RPYrot=RPYrot,
                         start_coords=self.positions[self.move_node[0]-1],
                         num_sides=num_sides)

        self.pos_hist = [self.positions.copy()]
        self.xd_hist = [np.zeros(self.positions.shape)]
        self.Ldot_hist = [np.zeros((self.edges_nodes.shape[0], 1))]
        self.L_hist = [self._calc_edge_lengths(self.positions)]
        self.thetad_hist = [np.zeros((self.L2th.shape[0], 1))]
        self.theta_hist = [np.zeros((self.L2th.shape[0], 1))]
        self.t_hist = [0]

        self.total_perimeters = self._calc_perimeters(self.L_hist[-1])
        self._scatter = None
        self._lines = []
        self._labels = []
        self._fills = []

    def _generate_embedding_shape(self, embedding_index) -> None:
        '''Generates the appropriate geometric information based on the user-specified embedding index'''
        self.supports: list[int] = self.data.support_move_dict[embedding_index]["support"]
        self.move_node: list[int] = self.data.support_move_dict[embedding_index]["move"]
        self.positions: np.ndarray = self._calc_init_node_positions(embedding_index)
        self.edges_nodes: np.ndarray  = self.data.edge_nodes_dict[embedding_index]
        self.triangle_nodes: np.ndarray  = self.data.triangle_nodes_dict[embedding_index]

        self.num_nodes = self.positions.shape[0]
        self.num_edges = self.edges_nodes.shape[0]
        self.num_triangles = self.triangle_nodes.shape[0]

        self._rotate_robot()
        triangle_incident_mat = np.array([[1, -1, 0],[0, 1, -1],[-1, 0, 1]])
        self.B_T = block_diag(*[triangle_incident_mat]*self.num_triangles)
        self.B_T = np.delete(self.B_T, [i for i in range(0, self.num_edges, 3)], axis=1)

        self.num_rollers = self.B_T.shape[1]

        self.L2th = np.linalg.pinv(self.B_T)

        self.R = self._calc_rigidity_matrix(self.positions)

        self.adj = self._calc_adj_matrix()

        self.triangle_loops = block_diag(*tuple([np.ones((1,3))]*self.num_triangles))
        self.support_indices = self._calc_support_indices()

    def _calc_edge_lengths(self, positions: np.ndarray):
        lengths = np.zeros((self.edges_nodes.shape[0], 1))

        positions_copy = positions.copy()

        for i, node_pairs in enumerate(self.edges_nodes):
            p1, p2 = node_pairs
            p1_idx = p1 - 1
            p2_idx = p2 - 1

            length = np.linalg.norm(positions_copy[p1_idx, :] - positions_copy[p2_idx, :])
            lengths[i, 0] = length

        return lengths

    def _calc_perimeters(self, edge_lengths: np.ndarray):
        return np.array([edge_lengths[i,0] + edge_lengths[i+1,0] + edge_lengths[i+2,0] for i in range(0, edge_lengths.shape[0], 3) ])

    def get_perimeters(self):
        return self._calc_perimeters(self.L_hist[-1])

    def get_perimeters_diff(self):
        return self._calc_perimeters(self.L_hist[-1]) - self.total_perimeters

    def _clean_matrix(self, matrix: np.ndarray):
        matrix_cp = matrix.copy()
        tolerance = 1e-12
        matrix_cp[np.abs(matrix) < tolerance] = 0
        return matrix_cp

    def _deconstruct_coords(self, coords):

        coords_list = [coords[:,n].ravel() for n in range(coords.shape[1])]

        return tuple(coords_list)

    def _calc_rigidity_matrix(self, positions: np.ndarray) -> np.ndarray:
        R = np.zeros((self.num_edges, self.num_nodes * self.dim))  # Initialize the rigidity matrix

        for i in range(self.num_edges):
            # Extract the points
            x1 = positions[self.edges_nodes[i, 0] - 1, :]
            x2 = positions[self.edges_nodes[i, 1] - 1, :]
            norm_x1x2 = np.linalg.norm(x1 - x2)

            # Loop over the dimensions
            for j in range(self.dim):
                R[i, self.edges_nodes[i, 0] - 1 + self.num_nodes * j] = (x1[j] - x2[j]) / norm_x1x2
                R[i, self.edges_nodes[i, 1] - 1 + self.num_nodes * j] = -R[i, self.edges_nodes[i, 0] - 1 + self.num_nodes * j]

        return R

    def get_R(self):
        return self.R

    def _calc_adj_matrix(self) -> np.ndarray:
        adj = np.zeros((self.num_nodes, self.num_nodes))
        for edge in self.edges_nodes:
            adj[edge[0]-1, edge[1]-1] = 1
        adj = adj + adj.T

        return adj

    def ol_update_and_store_positions_and_R(self, t, dt, xd: np.ndarray, Ldot: np.ndarray):
        self.t_hist.append(t+dt)
        xd = xd.reshape((-1, self.dim), order="F")
        delta_pos: np.ndarray = xd*dt

        self.positions += delta_pos.copy()

        self.pos_hist.append(self.positions.copy())
        self.xd_hist.append(xd.copy())
        self.Ldot_hist.append(Ldot.copy())
        self.thetad_hist.append((self.L2th@Ldot).copy())

        self.L_hist.append(self._calc_edge_lengths(self.positions))
        self.theta_hist.append(self.theta_hist[-1] + dt*self.thetad_hist[-1])

        self.R = self._calc_rigidity_matrix(self.positions)

    def _cl_update_positions_and_R(self, t, positions: np.ndarray, xd: np.ndarray, Ldot: np.ndarray, thetad: np.ndarray, L: np.ndarray, theta: np.ndarray):
        self.t_hist.append(t)

        self.pos_hist.append(positions.copy())
        self.xd_hist.append(xd.copy())
        self.Ldot_hist.append(Ldot.copy())
        self.thetad_hist.append(thetad.copy())

        self.L_hist.append(L.copy())
        self.theta_hist.append(theta.copy())

    def get_move_nodes_pos(self):
        return self.positions[self.move_node[0]-1, :]

    def update_plot(self):
        if self._scatter is None or not self._lines or not self._labels:
            raise RuntimeError("plot_robot must be called before update_plot")

        coords_xyz = self._deconstruct_coords(self.positions.copy())

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
        for edge, line in zip(self.edges_nodes, self._lines):
            p1, p2 = edge
            p1 -= 1
            p2 -= 1
            line.set_data([x[p1], x[p2]], [y[p1], y[p2]])
            if self.dim == 3:
                line.set_3d_properties([z[p1], z[p2]])

        # Update fills
        if self.dim == 2:
            for triangle, fill in zip(self.triangle_nodes, self._fills):
                p1, p2, p3 = triangle
                p1 -= 1
                p2 -= 1
                p3 -= 1
                fill.set_xy(np.array([[x[p1], y[p1]], [x[p2], y[p2]], [x[p3], y[p3]]]))

        # Update labels
        for label, xi, yi, zi in zip(self._labels, x, y, z):
            label.set_position((xi + 0.1, yi))
            if self.dim ==3:
                label.set_3d_properties(zi, zdir='z')

    def plot_dot(self, ax):
        dot, = ax.plot(*[[] for _ in range(self.dim)], 'o', color="blue")
        dot.set_markerfacecolor('blue')  # fill color
        dot.set_markeredgecolor('gray')  # optional edge color
        dot.set_markersize(8)

        return dot

    def update_dot(self, dot, position: np.ndarray):
        if self.dim == 2:
            dot.set_data([position[0]], [position[1]])
        elif self.dim == 3:
            dot.set_data_3d([position[0]], [position[1]], [position[2]])

    def update_arrow(self, ax, arrow: Quiver, position: np.ndarray, direction: np.ndarray):
        if arrow is not None:
            arrow.remove()
        return self.plot_arrow(ax, position, direction)

    def fk_position(self, t: float, dt: float, real_thetas: np.ndarray):
        real_thetas = real_thetas.reshape((-1, 1))
        average_theta_d = (real_thetas - self.theta_hist[-1]) / dt

        return self.fk_velocity(t, dt, average_theta_d)

    def fk_velocity(self, t: float, dt: float, real_thetads: np.ndarray):
        real_thetads = real_thetads.reshape((-1, 1))
        real_thetas = self.theta_hist[-1] + real_thetads*dt
        real_Ldot = self.B_T@real_thetads
        real_xd = self._convert_Ldot_to_xd(real_Ldot)
        real_xd = real_xd.reshape((-1, self.dim), order="F")
        self.positions = self.positions + real_xd*dt
        real_L = self._calc_edge_lengths(self.positions)
        self.R = self._calc_rigidity_matrix(self.positions)
        self._cl_update_positions_and_R(t, self.positions, real_xd, real_Ldot, real_thetads, real_L, real_thetas)

    def convert_xd_to_thetad(self, xd: np.ndarray) -> np.ndarray:
        xd = xd.reshape((-1, 1))
        Ldot = self.R@xd
        thetad = self.L2th@Ldot
        return thetad.reshape((1, -1))

    def _convert_Ldot_to_xd(self, Ldot: np.ndarray) -> np.ndarray:
        xd = np.zeros((self.num_nodes*self.dim, 1))
        R_reduced = np.delete(self.R, self.support_indices, axis=1)
        xd_reduced = np.linalg.inv(R_reduced)@Ldot

        value_position = [i for i in range(xd.shape[0]) if i not in self.support_indices]
        xd[value_position, 0] = xd_reduced[:, 0]

        return xd

    def _calc_support_indices(self):
        support_indices = []
        for i, support in enumerate(self.supports):
            for d in range(i, self.dim):
                support_indices.append((support - 1) + d*self.num_nodes)
        return sorted(support_indices)

    @abstractmethod
    def _rotate_robot(self) -> None:
        '''Rotates robot positions such that supports are on xy plane and origin is at first point'''

    @abstractmethod
    def plot_robot(self, ax) -> None:
        pass

    @abstractmethod
    def plot_path(self, ax, fill=True) -> None:
        pass

    @abstractmethod
    def create_fig_ax(self, equal_aspect=True) -> tuple[Figure, Axes | Axes3D, Axes]:
        """Create a figure and return (fig, main_axis_for_robot, axis_for_theta_history)

        The extra axis is used by MotionViz to plot theta histories alongside the
        robot visualization.
        """
        pass

    @abstractmethod
    def show(self, ax) -> None:
        pass

    @abstractmethod
    def plot_arrow(self, ax, position: np.ndarray, direction: np.ndarray):
        pass

    @abstractmethod
    def _calc_init_node_positions(self, embedding_index):
        pass

class Robot2D(TrussRobot):

    def __init__(self, index: int, path_type: str = "square", RPYrot: list[float] = [0.], path_scale: float = 1, num_sides: int = 4):
        """
        Initialize the object with specified parameters.
        Args:
            index (int): The index of the object.
            path_type (str, optional): The type of path to use. Defaults to "square".
            RPYrot (list[float], optional): List containing a single rotation angle (Roll-Pitch-Yaw). Defaults to [0.].
            path_scale (float, optional): Scale factor for the path. Defaults to 1.
            num_sides (int, optional): Number of sides for the path shape. Defaults to 4.
        Raises:
            Exception: If more than one rotation angle is specified in RPYrot.
        """

        if len(RPYrot) != 1:
            raise Exception("Only one rotation angle can be specified")
        super().__init__(index=index, dimension=2, path_type=path_type, RPYrot=RPYrot, path_scale=path_scale, num_sides=num_sides)

    def _rotate_robot(self) -> None:
        orign_node_idx = self.supports[0] - 1
        linear_node_idx = self.supports[1] - 1

        self.positions -= self.positions[orign_node_idx]
        theta = -np.arctan2(self.positions[linear_node_idx][1] - self.positions[orign_node_idx][1],
                           self.positions[linear_node_idx][0] - self.positions[orign_node_idx][0])

        self.positions = (rot2D(theta)@(self.positions.T)).T

    def _calc_init_node_positions(self, embedding_index):
        return self.data.initial_node_positions[embedding_index]

    def plot_robot(self, ax: Axes):

        x, y = self._deconstruct_coords(self.positions.copy())

        # Plot the points
        self._scatter = ax.scatter(x, y, color='b', s=50, label='Points')

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, edge in enumerate(self.edges_nodes):
            p1, p2 = edge
            p1 -= 1
            p2 -= 1
            line, = ax.plot([x[p1], x[p2]], [y[p1], y[p2]], triangle_colors[idx // 3], lw=3)
            self._lines.append(line)

        for idx, triangle in enumerate(self.triangle_nodes):
            p1, p2, p3 = triangle
            p1 -= 1
            p2 -= 1
            p3 -= 1
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

    def plot_path(self, ax: Axes, fill=True):
        x, y = self._deconstruct_coords(self.path.transformed_path)
        ax.plot(x, y)

        if fill:
            ax.fill(x, y, facecolor='red', alpha=0.5, hatch='--')

        self.path_plotted = True

    def create_fig_ax(self) -> tuple[Figure, Axes, Axes]:
        # Create a two-panel figure: left for robot, right for theta history
        fig, (ax_robot, ax_theta) = plt.subplots(ncols=2, figsize=(10, 5))
        return fig, ax_robot, ax_theta

    def show(self, ax: Axes):
        if not self.robot_plotted and not self.path_plotted:
            self.robot_plotted = False
            self.path_plotted = False
            plt.show()
            return

        xs, ys = [], []

        if self.robot_plotted:
            x, y = self._deconstruct_coords(self.positions)
            xs.append(x)
            ys.append(y)

        if self.path_plotted:
            x, y = self._deconstruct_coords(self.path.transformed_path)
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

    def plot_arrow(self, ax, position: np.ndarray, direction: np.ndarray):
        return ax.quiver(*position.tolist(), *direction.tolist(), color='blue', linewidth=3)

class Robot3D(TrussRobot):

    def __init__(self, index: int, path_type: str = "square", RPYrot: list[float] = [0., 0., 0.], path_scale: float = 1, num_sides: int = 4):
        """
        Initialize the object with specified parameters.
        Args:
            index (int): Unique identifier for the object.
            path_type (str, optional): Type of path to use. Defaults to "square".
            RPYrot (list[float], optional): List of 3 Euler angles (roll, pitch, yaw) for rotation. Defaults to [0., 0., 0.].
            path_scale (float, optional): Scaling factor for the path. Defaults to 1.
            num_sides (int, optional): Number of sides for the path shape. Defaults to 4.
        Raises:
            Exception: If RPYrot does not contain exactly 3 elements.
        """

        if len(RPYrot) != 3:
            raise Exception("3 Euler angles required")
        super().__init__(index=index, dimension=3, path_type=path_type, RPYrot=RPYrot, path_scale=path_scale, num_sides=num_sides)

    def _rotate_robot(self):
        '''Rotates robot positions such that supports are on xy plane and origin is at first point'''
        origin_node_idx = self.supports[0] - 1 # This node should be centered at origin
        linear_node_idx = self.supports[1] - 1 # This node should be aligned along the x axis
        free_node_idx = self.supports[2] - 1 # This node should be on xy plane

        self.positions -= self.positions[origin_node_idx]
        theta_z = -np.arctan2(self.positions[linear_node_idx][1] - self.positions[origin_node_idx][1],
                              self.positions[linear_node_idx][0] - self.positions[origin_node_idx][0])
        self.positions = (rotz(theta_z)@(self.positions.T)).T
        theta_y = np.arctan2(self.positions[linear_node_idx, 2], self.positions[linear_node_idx, 0])
        self.positions = (roty(theta_y)@(self.positions.T)).T
        theta_x = -np.arctan2(self.positions[free_node_idx][2], self.positions[free_node_idx][1])
        self.positions = (rotx(theta_x)@(self.positions.T)).T
        # self.positions = self._clean_matrix(self.positions)

        if np.sum(self.positions[:, -1]) < 0:
            self.positions[:,-1] *= -1

    def _calc_init_node_positions(self, embedding_index):
        return self.data.initial_node_positions[embedding_index]

    def plot_robot(self, ax: Axes3D):

        x, y, z = self._deconstruct_coords(self.positions.copy())

        # Plot the points
        self._scatter = ax.scatter(x, y, z, color='b', s=50, label='Points')

        for line in self._lines:
            line.remove()
        self._lines.clear()

        # Plot the edges
        triangle_colors = {0: 'b-', 1: 'r-', 2: 'k-', 3: 'g-', 4: 'c-', 5:'m-', 6: 'y-'}
        for idx, edge in enumerate(self.edges_nodes):
            p1, p2 = edge
            p1 -= 1
            p2 -= 1
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

    def plot_path(self, ax: Axes3D, fill=True):
        x, y, z = self._deconstruct_coords(self.path.transformed_path)
        ax.scatter(x, y, z, color='r')

        if fill:
            verts = [list(zip(x, y, z))]
            poly = Poly3DCollection(verts, alpha=0.5, facecolor='red')
            ax.add_collection3d(poly)

        self.path_plotted = True

    def create_fig_ax(self, equal_aspect=True) -> tuple[Figure, Axes3D, Axes]:
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
            x, y, z = self._deconstruct_coords(self.positions)
            xs.append(x)
            ys.append(y)
            zs.append(z)

        if self.path_plotted:
            x, y, z = self._deconstruct_coords(self.path.transformed_path)
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

    def plot_arrow(self, ax, position: np.ndarray, direction: np.ndarray):
        return ax.quiver(*position.tolist(), *direction.tolist(), color='blue', length=1.0, normalize=True, linewidth=6, arrow_length_ratio=0.4)

if __name__ == "__main__":
    # robot = Robot2D(1, ZYZrot=[0.])
    robot = Robot3D(1, RPYrot=[45, 30, 45])

    fig, ax_robot, ax_theta = robot.create_fig_ax()
    robot.plot_robot(ax_robot)
    robot.plot_path(ax_robot)
    robot.show(ax_robot)