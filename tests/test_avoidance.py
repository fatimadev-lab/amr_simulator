"""
Unit tests for the ObstacleAvoider module.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from avoidance import ObstacleAvoider


def test_avoider_initialization():
    """Test ObstacleAvoider initialization."""
    avoider = ObstacleAvoider(safety_radius=0.5)
    assert avoider.safety_radius == 0.5


def test_no_collision():
    """Test when there is no collision."""
    avoider = ObstacleAvoider(safety_radius=0.5)
    
    pose = (0.0, 0.0, 0.0)
    ranges = np.array([1.0, 1.0, 1.0, 1.0])
    angles = np.array([-0.5, -0.1, 0.1, 0.5])
    
    collision = avoider.check_immediate_collision(pose, ranges, angles)
    assert not collision


def test_collision_in_front():
    """Test collision detection in front of robot."""
    avoider = ObstacleAvoider(safety_radius=0.5)
    
    pose = (0.0, 0.0, 0.0)
    # Obstacle directly in front
    ranges = np.array([1.0, 0.3, 0.2, 1.0])
    angles = np.array([-0.5, -0.1, 0.1, 0.5])
    
    collision = avoider.check_immediate_collision(pose, ranges, angles)
    assert collision


def test_collision_behind():
    """Test no collision when obstacle is behind robot."""
    avoider = ObstacleAvoider(safety_radius=0.5)
    
    pose = (0.0, 0.0, 0.0)
    # Obstacles to the sides/behind
    ranges = np.array([0.2, 1.0, 1.0, 0.3])
    angles = np.array([-np.pi/2, -0.1, 0.1, np.pi/2])
    
    collision = avoider.check_immediate_collision(pose, ranges, angles)
    # Should not detect collision if obstacles are not in front cone
    # Front cone is defined by cos(angles) > 0.5


def test_find_blocking_beams():
    """Test finding blocking beam indices."""
    avoider = ObstacleAvoider(safety_radius=0.5)
    
    ranges = np.array([1.0, 0.3, 0.2, 1.0, 0.4])
    angles = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
    threshold = 0.5
    
    blocking = avoider.find_blocking_beams(ranges, angles, threshold)
    assert len(blocking) > 0
    assert 1 in blocking  # ranges[1] = 0.3 < 0.5
    assert 2 in blocking  # ranges[2] = 0.2 < 0.5
    assert 4 in blocking  # ranges[4] = 0.4 < 0.5
    assert 0 not in blocking  # ranges[0] = 1.0 >= 0.5


def test_different_safety_radius():
    """Test with different safety radius."""
    avoider = ObstacleAvoider(safety_radius=1.0)
    
    pose = (0.0, 0.0, 0.0)
    ranges = np.array([1.0, 0.8, 1.0, 1.0])
    angles = np.array([-0.5, 0.0, 0.1, 0.5])
    
    collision = avoider.check_immediate_collision(pose, ranges, angles)
    # With safety_radius=1.0, obstacle at 0.8m in front cone should trigger collision
    assert collision


if __name__ == "__main__":
    test_avoider_initialization()
    test_no_collision()
    test_collision_in_front()
    test_collision_behind()
    test_find_blocking_beams()
    test_different_safety_radius()
    print("All avoidance tests passed!")
