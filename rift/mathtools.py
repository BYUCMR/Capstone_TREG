import numpy as np
from scipy.spatial import ConvexHull


def point_to_segment_distance(p, a, b):
    """Return distance and closest point from p to segment ab."""
    ab = b - a
    denom = np.dot(ab, ab)
    if denom == 0:
        return np.linalg.norm(p - a), a
    t = np.dot(p - a, ab) / denom
    t = np.clip(t, 0, 1)
    closest = a + t * ab
    dist = np.linalg.norm(p - closest)
    return dist, closest


def check_inside_and_closest_edge(p, points_2d, *, tol=1e-12):
    """Check if point is inside convex hull and find closest edge if outside."""
    hull = ConvexHull(points_2d)
    A = hull.equations[:, :2]
    c = hull.equations[:, 2]
    vals = A @ p + c  # positive => outside that edge

    max_violation = np.max(vals)
    is_inside = max_violation <= tol
    signed_distance = max_violation

    if is_inside:
        return True, signed_distance, None, None

    hv = hull.vertices
    n = len(hv)

    best_dist = np.inf
    best_edge = None
    best_closest = None
    for fi in range(n):
        i1, i2 = hv[fi], hv[(fi + 1) % n]
        a, b = points_2d[i1], points_2d[i2]
        d, cpt = point_to_segment_distance(p, a, b)
        if d < best_dist:
            best_dist, best_edge, best_closest = d, (i1, i2), cpt

    return False, signed_distance, best_edge, best_closest


def rotate_points_about_line(points, line_point, line_dir, angle):
    points = np.array(points)
    line_point = np.array(line_point)
    line_dir = np.array(line_dir) / np.linalg.norm(line_dir)
    theta = angle

    def rotate_single(p):
        v = line_dir
        p_rel = p - line_point
        return (p_rel * np.cos(theta)
                + np.cross(v, p_rel) * np.sin(theta)
                + v * np.dot(v, p_rel) * (1 - np.cos(theta))
                ) + line_point

    return np.array([rotate_single(p) for p in points])


def find_point_plane_angle(v1, v2, p1, p2):
    def vector_projection(a, b):
        if np.linalg.norm(b) == 0:
            raise ValueError("Cannot project onto a zero vector.")

        scalar_projection = np.dot(a, b) / np.dot(b, b)
        projection_vector = scalar_projection * b
        return projection_vector

    w = p2 - p1
    proj_vec = vector_projection(w, v1)
    p_new = p1 + proj_vec
    w1 = p2 - p_new
    theta = np.arccos(np.dot(w1, v2) / (np.linalg.norm(w1) * np.linalg.norm(v2)))
    return theta


def ground_plane(support):
    v1 = support[1] - support[0]
    v2 = support[2] - support[0]
    normal = np.cross(v1, v2)
    offset = np.dot(support[0], normal)
    normal /= np.linalg.norm(normal)
    return normal, offset
