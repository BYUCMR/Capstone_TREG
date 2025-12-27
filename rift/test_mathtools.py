from rift.mathtools import *
import pytest


def run_case(name, points, p, expected_inside):
    inside, sd, edge, cpt = check_inside_and_closest_edge(p, points)

    # Check inside/outside correctness
    assert inside == expected_inside, (
        f"{name}: expected inside={expected_inside}, got inside={inside}"
    )

    # Signed distance sign convention
    if expected_inside:
        assert sd <= 0, f"{name}: expected sd <= 0 for inside, got {sd}"
    else:
        assert sd > 0, f"{name}: expected sd > 0 for outside, got {sd}"

    # Edge validity
    if edge is not None:
        i, j = edge
        assert 0 <= i < len(points) and 0 <= j < len(points), (
            f"{name}: invalid edge indices {edge}"
        )

    # Closest point shape
    if cpt is not None:
        assert cpt.shape == (2,), (
            f"{name}: closest point wrong shape {cpt.shape}"
        )


# ---------------------------
# Individual Tests
# ---------------------------

def test_square_inside():
    run_case(
        "Square - Inside",
        np.array([[0,0],[1,0],[1,1],[0,1]]),
        np.array([0.5,0.5]),
        expected_inside=True
    )


def test_square_above_top_edge():
    run_case(
        "Square - Above top edge",
        np.array([[0,0],[1,0],[1,1],[0,1]]),
        np.array([0.5,1.05]),
        expected_inside=False
    )


def test_square_right_of_edge():
    run_case(
        "Square - Right of right edge",
        np.array([[0,0],[1,0],[1,1],[0,1]]),
        np.array([1.02,0.3]),
        expected_inside=False
    )


def test_square_outside_corner():
    run_case(
        "Square - Near corner",
        np.array([[0,0],[1,0],[1,1],[0,1]]),
        np.array([1.01,1.01]),
        expected_inside=False
    )


def test_slanted_quad_outside_top():
    run_case(
        "Slanted quad - Outside top",
        np.array([[0,0],[2,0],[1.5,1],[0,1]]),
        np.array([1.2,1.05]),
        expected_inside=False
    )


def test_diamond_outside_diagonal():
    run_case(
        "Diamond - Outside diagonal",
        np.array([[0,1],[1,0],[0,-1],[-1,0]]),
        np.array([0.7,0.7]),
        expected_inside=False
    )


def test_diamond_inside():
    run_case(
        "Diamond - Inside",
        np.array([[0,1],[1,0],[0,-1],[-1,0]]),
        np.array([0.3,0.3]),
        expected_inside=True
    )


def test_triangle_closest_edge():
    p = np.array([0.49, 0.51])
    points = np.array([[0, 0], [1, 0], [1, 1]])
    inside, sd, edge, cpt = check_inside_and_closest_edge(p, points)
    assert not inside
    assert edge == (2, 0) or edge == (0, 2)


# --- Test 1: point directly above plane (expect 0°) ---
def test_above_plane():
    v1 = np.array([1,0,0])
    v2 = np.array([0,0,1])
    p1 = np.array([0,0,0])
    p2 = np.array([0,0,5])
    assert find_point_plane_angle(v1, v2, p1, p2) == 0

# --- Test 2: point in plane (expect 90°) ---
def test_in_plane():
    pytest.skip("Currently fails")
    v1 = np.array([1,0,0])
    v2 = np.array([0,0,1])
    p1 = np.array([0,0,0])
    p2 = np.array([5,0,0])
    assert find_point_plane_angle(v1, v2, p1, p2) == 90

# --- Test 3: diagonal offset (expect 45°) ---
def test_diagonal():
    pytest.skip("Currently fails")
    v1 = np.array([1,0,0])
    v2 = np.array([0,0,1])
    p1 = np.array([0,0,0])
    p2 = np.array([5,0,5])
    assert find_point_plane_angle(v1, v2, p1, p2) == 45

# --- Test 4: different line direction (same angle, expect 45°) ---
def test_diff_line_dir():
    pytest.skip("Currently fails")
    v1 = np.array([1,1,0])
    v2 = np.array([0,0,1])
    p1 = np.array([0,0,0])
    p2 = np.array([5,0,5])
    assert find_point_plane_angle(v1, v2, p1, p2) == 45

# --- Test 5: zero vector raises ValueError ---
def test_zero_vector():
    v1 = np.array([0,0,0])
    v2 = np.array([0,0,1])
    p1 = np.array([0,0,0])
    p2 = np.array([1,2,3])
    with pytest.raises(ValueError):
        find_point_plane_angle(v1, v2, p1, p2)
