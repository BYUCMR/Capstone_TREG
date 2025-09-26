import numpy as np
from util import rotx, roty, rotz, rot2D

class Path():
    def __init__(self, type: str, dimension: int, RPYrot: list[float], start_coords: np.ndarray, length: float = 1, num_sides: int = 4) -> None:
        """
        Initialize a path object with specified parameters.
        Args:
            type (str): The type of path to generate. Options include "line", "circle", "polygon", "inscribed_polygon", and "thin_byu".
            dimension (int): The number of spatial dimensions for the path (e.g., 2 or 3).
            RPYrot (list[float]): List of rotation angles in degrees (Roll, Pitch, Yaw) to apply to the path. If not provided, defaults to zeros.
            start_coords (np.ndarray): The starting coordinates for the path as a NumPy array.
            length (float, optional): The length or size parameter for the path. Defaults to 1.
            num_sides (int, optional): Number of sides for polygonal paths. Defaults to 4.
        Returns:
            None
        """
        
        options = {
            "line": self._generate_line_path,
            "circle": self._generate_circle_path,
            "polygon": self._generate_polygon_path,
            "inscribed_polygon": self._generate_inscribed_polygon_path,
            "thin_byu": self._generate_thin_y_path
        }

        self.d = dimension

        if not RPYrot:
            RPYrot = [0.]*self.d

        RPYrot = [np.deg2rad(angle) for angle in RPYrot]

        self.raw_path: np.ndarray = options[type](length, num_sides)
        self.transformed_path: np.ndarray = self._calc_transformed_path(RPYrot, start_coords)

    def _generate_polygon_path(self, length: float, num_sides: int) -> np.ndarray:
        """Generates a polygon with side length of "length"

        Args:
            length (float): The length of the side
            num_sides (int): The number of sides in the polygon

        Returns:
            np.ndarray: Array of goal coordinates, with shape (num_sides+1, dimension)
        """

        if num_sides < 3:
            raise Exception("Polygon path must have three or more sides")

        rot_angle_for_each_side = 2*np.pi / num_sides
        path = np.zeros((num_sides + 1, self.d))

        R_func = self._get_rotation_matrix_func()

        path[1,0] = length

        for i in range(2, num_sides+1):
            path[i,:] = (R_func(rot_angle_for_each_side*(i-1))@(path[1,:]).T) + path[i-1,:]

        return self._clean_matrix(path)

    def _generate_inscribed_polygon_path(self, length: float, num_sides: int) -> np.ndarray:
        """Generates a polygon inscribed in an imaginary circle with diameter "length"

        Args:
            length (float): The radius of the circumscribing circle
            num_sides (int): The number of sides in the polygon

        Returns:
            np.ndarray: Array of goal coordinates, with shape (num_sides+1, dimension)
        """

        if num_sides < 3:
            raise Exception("Polygon path must have three or more sides")

        rot_angle_for_each_side = 2*np.pi / num_sides
        path = np.zeros((num_sides + 1, self.d))

        R_func = self._get_rotation_matrix_func()

        angle_from_base_corner_to_centroid = (np.pi - rot_angle_for_each_side) / 2.0

        initial_radius_vector = np.zeros((1,self.d))
        initial_radius_vector[0,0] = length / 2.0
        vector_from_corner_to_centroid = (R_func(angle_from_base_corner_to_centroid)@initial_radius_vector.T).T

        vector_from_centroid_to_corner = -vector_from_corner_to_centroid

        for i in range(1, num_sides+1):
            path[i,:] = vector_from_corner_to_centroid + (R_func(rot_angle_for_each_side*i)@vector_from_centroid_to_corner.T).T

        return self._clean_matrix(path)


    def _generate_line_path(self, length: float, _) -> np.ndarray:
        """Generates a line, with each row representing a goal coordinate

        Args:
            length (float): Total length of the line

        Returns:
            np.ndarray: Array of goal coordinates, with shape (4, dimension)
        """
        path = np.zeros((4, self.d))
        path[1,0] = length / 2.0
        path[2,0] = -length / 2.0

        return self._clean_matrix(path)

    def _generate_circle_path(self, length: float, _) -> np.ndarray:
        """Generates a discrete circle, with each row representing a goal coordinate

        Args:
            length (float): The diameter of the circle

        Returns:
            np.ndarray: Array of goal coordinates, with shape (4, dimension)
        """

        return self._generate_inscribed_polygon_path(length, 20)

    def _generate_thin_y_path(self, length: float, _) -> np.ndarray:
        """Generates a thin y path

        Args:
            length (float): The relative scaling of the y (the bottom half is of size "length")
            _ (_type_): A throwaway parameter from the dictionary-based function calls

        Returns:
            np.ndarray: Array of goal coordinates, with shape (7, dimension)
        """
        path = np.zeros((7, self.d))

        path[1,0:2] = np.array([[0, length]])
        path[2,0:2] = np.array([[length*np.sin(np.deg2rad(40)), length + length*np.cos(np.deg2rad(40))]])
        path[3,0:2] = np.array([[0, length]])
        path[4,0:2] = np.array([[-length*np.sin(np.deg2rad(40)), length + length*np.cos(np.deg2rad(40))]])
        path[5,0:2] = np.array([[0, length]])

        return path

    def _calc_transformed_path(self, RPYrot, start_coords) -> np.ndarray:

        if self.d == 3:
            self.R = rotz(RPYrot[0])@roty(RPYrot[1])@rotx(RPYrot[2])
        else:
            self.R = rot2D(RPYrot[0])

        return self._clean_matrix((self.R@self.raw_path.T).T + start_coords)

    def _get_rotation_matrix_func(self):
        return rotz if self.d == 3 else rot2D

    def _clean_matrix(self, matrix):
        return np.where(np.abs(matrix) < 1e-10, 0, matrix)