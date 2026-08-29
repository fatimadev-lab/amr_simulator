"""
Unit tests for the Simulator module.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from simulator import Simulator


def test_simulator_initialization():
    """Test Simulator initialization."""
    true_map = np.zeros((100, 100), dtype=np.uint8)
    sim = Simulator(true_map, resolution=0.1, lidar_range=6.0, n_beams=108)
    
    assert sim.resolution == 0.1
    assert sim.lidar_range == 6.0
    assert sim.n_beams == 108
    assert len(sim.angles) == 108


def test_world_to_map():
    """Test coordinate conversion from world to map."""
    true_map = np.zeros((100, 100), dtype=np.uint8)
    sim = Simulator(true_map, resolution=0.1)
    
    # Test conversion
    ix, iy = sim.world_to_map(5.0, 5.0)
    assert ix == 50
    assert iy == 50


def test_map_to_world():
    """Test coordinate conversion from map to world."""
    true_map = np.zeros((100, 100), dtype=np.uint8)
    sim = Simulator(true_map, resolution=0.1)
    
    # Test conversion
    x, y = sim.map_to_world(50, 50)
    assert x == 5.0
    assert y == 5.0


def test_get_lidar():
    """Test LiDAR scan generation."""
    true_map = np.zeros((200, 200), dtype=np.uint8)
    # Add border walls
    true_map[0, :] = 1
    true_map[-1, :] = 1
    true_map[:, 0] = 1
    true_map[:, -1] = 1
    
    sim = Simulator(true_map, resolution=0.1, lidar_range=5.0, n_beams=36)
    pose = (1.0, 1.0, 0.0)
    ranges = sim.get_lidar(pose)
    
    assert len(ranges) == 36
    assert np.all(ranges >= 0)
    assert np.all(ranges <= 5.0)


def test_step_pose():
    """Test pose update with kinematics."""
    true_map = np.zeros((100, 100), dtype=np.uint8)
    sim = Simulator(true_map, resolution=0.1, odom_noise=0.0)
    
    pose = (0.0, 0.0, 0.0)
    # Move forward
    new_pose = sim.step_pose(pose, v=1.0, w=0.0, dt=0.1)
    
    # Should move approximately 0.1 meters forward
    assert new_pose[0] > 0.05  # Accounting for noise
    assert abs(new_pose[2]) < 0.01  # Yaw should not change much


if __name__ == "__main__":
    test_simulator_initialization()
    test_world_to_map()
    test_map_to_world()
    test_get_lidar()
    test_step_pose()
    print("All simulator tests passed!")
