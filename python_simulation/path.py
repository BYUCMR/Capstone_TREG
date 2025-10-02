import numpy as np
from linalg import Matrix, rotx, roty, rotz, rot2D, clean_matrix


def transform_path(
    shape: str,
    dimension: int,
    RPYrot: tuple[float, ...] | None = None,
    start_coords: Matrix | None = None,
    length: float = 1,
    num_sides: int = 4,
) -> Matrix:
    """
    Initialize a path object with specified parameters.
    Args:
        shape (str): The type of path to generate. Options include "line", "circle", "polygon", "inscribed_polygon", and "thin_byu".
        dimension (int): The number of spatial dimensions for the path (i.e., 2 or 3).
        RPYrot (tuple[float, ...], optional): Tuple of rotation angles in degrees (Roll, Pitch, Yaw) to apply to the path.
        start_coords (np.ndarray, optional): The starting coordinates for the path as a NumPy array.
        length (float, optional): The length or size parameter for the path. Defaults to 1.
        num_sides (int, optional): Number of sides for polygonal paths. Defaults to 4.
    Returns:
        np.ndarray: The transformed path.
    """
    if shape == "line":
        path = _generate_line_path(dimension, length)
    elif shape == "circle":
        path = _generate_inscribed_polygon_path(dimension, length, 20)
    elif shape == "polygon":
        path = _generate_polygon_path(dimension, length, num_sides)
    elif shape == "inscribed_polygon":
        path = _generate_inscribed_polygon_path(dimension, length, num_sides)
    elif shape == "thin_byu":
        path = _generate_thin_y_path(dimension, length)
    else:
        raise ValueError(f"{shape!r} is not a supported path type")

    if dimension == 2:
        if RPYrot is None:
            r = 0
        else:
            r, = (np.deg2rad(angle) for angle in RPYrot)
        xform = rot2D(r)
    elif dimension == 3:
        if RPYrot is None:
            r, p, y = 0, 0, 0
        else:
            r, p, y = (np.deg2rad(angle) for angle in RPYrot)
        xform = rotz(r) @ roty(p) @ rotx(y)
    else:
        raise ValueError(f"{dimension!r} is not a valid number of spatial dimensions")
    if start_coords is None:
        start_coords = np.zeros(path.shape)
    transformed_path = (xform @ path.T).T + start_coords

    return clean_matrix(transformed_path)


def _generate_polygon_path(dim: int, length: float, num_sides: int) -> Matrix:
    """Generates a polygon with side length of "length"

    Args:
        dim (int)
        length (float): The length of the side
        num_sides (int): The number of sides in the polygon

    Returns:
        np.ndarray: Array of goal coordinates, with shape (num_sides+1, dim)
    """
    if num_sides < 3:
        raise Exception("Polygon path must have three or more sides")

    rot_angle_for_each_side = 2*np.pi / num_sides
    path = np.zeros((num_sides + 1, dim))

    rot_func = rotz if dim == 3 else rot2D

    path[1,0] = length

    for i in range(2, num_sides+1):
        path[i,:] = (rot_func(rot_angle_for_each_side*(i-1))@(path[1,:]).T) + path[i-1,:]

    return clean_matrix(path)


def _generate_inscribed_polygon_path(dim: int, length: float, num_sides: int) -> Matrix:
    """Generates a polygon inscribed in an imaginary circle with diameter "length"

    Args:
        dim (int)
        length (float): The radius of the circumscribing circle
        num_sides (int): The number of sides in the polygon

    Returns:
        np.ndarray: Array of goal coordinates, with shape (num_sides+1, dim)
    """
    if num_sides < 3:
        raise Exception("Polygon path must have three or more sides")

    rot_angle_for_each_side = 2*np.pi / num_sides
    path = np.zeros((num_sides + 1, dim))

    rot_func = rotz if dim == 3 else rot2D

    angle_from_base_corner_to_centroid = (np.pi - rot_angle_for_each_side) / 2.0

    initial_radius_vector = np.zeros((1, dim))
    initial_radius_vector[0,0] = length / 2.0
    vector_from_corner_to_centroid = (rot_func(angle_from_base_corner_to_centroid)@initial_radius_vector.T).T

    vector_from_centroid_to_corner = -vector_from_corner_to_centroid

    for i in range(1, num_sides+1):
        path[i,:] = vector_from_corner_to_centroid + (rot_func(rot_angle_for_each_side*i)@vector_from_centroid_to_corner.T).T

    return clean_matrix(path)


def _generate_line_path(dim: int, length: float) -> Matrix:
    """Generates a line, with each row representing a goal coordinate

    Args:
        dim (int)
        length (float): Total length of the line

    Returns:
        np.ndarray: Array of goal coordinates, with shape (4, dim)
    """
    path = np.zeros((4, dim))
    path[1,0] = length / 2.0
    path[2,0] = -length / 2.0

    return clean_matrix(path)


def _generate_thin_y_path(dim: int, length: float) -> Matrix:
    """Generates a thin y path

    Args:
        dim (int)
        length (float): The relative scaling of the y (the bottom half is of size "length")

    Returns:
        np.ndarray: Array of goal coordinates, with shape (7, dim)
    """
    path = np.zeros((7, dim))

    path[1,0:2] = np.array([[0, length]])
    path[2,0:2] = np.array([[length*np.sin(np.deg2rad(40)), length + length*np.cos(np.deg2rad(40))]])
    path[3,0:2] = np.array([[0, length]])
    path[4,0:2] = np.array([[-length*np.sin(np.deg2rad(40)), length + length*np.cos(np.deg2rad(40))]])
    path[5,0:2] = np.array([[0, length]])

    return path
