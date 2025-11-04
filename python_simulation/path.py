import numpy as np

from linalg import Matrix, rot2D, rotz


def make_path(
    *,
    shape: str = 'polygon',
    dimension: int = 3,
    xform: Matrix = np.eye(3),
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
        xform = xform[0:2, 0:2]
    elif dimension != 3:
        raise ValueError(f"{dimension!r} is not a valid number of spatial dimensions")

    return np.matvec(xform, path)


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
    corner_angle = 2*np.pi / num_sides
    path = np.zeros((num_sides + 1, dim))
    path[1, 0] = length
    rot_func = rotz if dim == 3 else rot2D
    for i in range(2, num_sides+1):
        xform = rot_func(corner_angle * (i-1))
        path[i,:] = xform @ (path[1,:]).T + path[i-1,:]
    return path


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
    corner_angle = 2*np.pi / num_sides
    rot_func = rotz if dim == 3 else rot2D
    corner = np.zeros(dim)
    corner[0] = -length / 2
    path = np.vstack([rot_func(np.pi/2 + corner_angle*(i-0.5)) @ corner for i in range(num_sides+1)])
    path -= path[0]
    return path


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
    return path


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
