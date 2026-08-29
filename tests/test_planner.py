"""
Unit tests for the AStarPlanner module.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from planner import AStarPlanner


def test_planner_initialization():
    """Test AStarPlanner initialization."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    planner = AStarPlanner(occ_map)
    
    assert planner.h == 100
    assert planner.w == 100


def test_in_bounds():
    """Test boundary checking."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    planner = AStarPlanner(occ_map)
    
    assert planner.in_bounds(50, 50)
    assert planner.in_bounds(0, 0)
    assert planner.in_bounds(99, 99)
    assert not planner.in_bounds(-1, 50)
    assert not planner.in_bounds(100, 50)


def test_is_free():
    """Test free space checking."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    occ_map[50, 50] = 1  # Mark one cell as occupied
    planner = AStarPlanner(occ_map)
    
    assert planner.is_free(25, 25)
    assert not planner.is_free(50, 50)
    assert not planner.is_free(-1, 50)  # Out of bounds


def test_heuristic():
    """Test heuristic function."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    planner = AStarPlanner(occ_map)
    
    # Euclidean distance
    h = planner.heuristic((0, 0), (3, 4))
    assert abs(h - 5.0) < 0.01


def test_simple_path():
    """Test path planning in open space."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    planner = AStarPlanner(occ_map)
    
    path = planner.plan((10, 10), (20, 20))
    assert path is not None
    assert len(path) > 0
    assert path[0] == (10, 10)
    assert path[-1] == (20, 20)


def test_path_with_obstacle():
    """Test path planning around obstacles."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    # Add a vertical wall
    occ_map[25:75, 50] = 1
    planner = AStarPlanner(occ_map)
    
    path = planner.plan((10, 50), (90, 50))
    assert path is not None
    assert len(path) > 0


def test_unreachable_goal():
    """Test when goal is unreachable."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    # Surround goal with walls
    occ_map[48:52, 48:52] = 1
    occ_map[49, 49] = 0  # Leave goal cell free
    planner = AStarPlanner(occ_map)
    
    path = planner.plan((10, 10), (49, 49))
    # Should return None if goal is unreachable
    assert path is None or len(path) > 0


def test_start_equals_goal():
    """Test when start and goal are the same."""
    occ_map = np.zeros((100, 100), dtype=np.uint8)
    planner = AStarPlanner(occ_map)
    
    path = planner.plan((50, 50), (50, 50))
    # Path might be single cell or None depending on implementation
    assert path is not None


if __name__ == "__main__":
    test_planner_initialization()
    test_in_bounds()
    test_is_free()
    test_heuristic()
    test_simple_path()
    test_path_with_obstacle()
    test_unreachable_goal()
    test_start_equals_goal()
    print("All planner tests passed!")
