"""
Unit tests for the OccupancyGridMapper module.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from mapping import OccupancyGridMapper


def test_mapper_initialization():
    """Test OccupancyGridMapper initialization."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    assert mapper.size_x == 10.0
    assert mapper.size_y == 10.0
    assert mapper.resolution == 0.1
    assert mapper.width == 100
    assert mapper.height == 100


def test_world_to_cell():
    """Test coordinate conversion from world to cell."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    ix, iy = mapper.world_to_cell(5.0, 5.0)
    assert ix == 50
    assert iy == 50


def test_cell_to_world():
    """Test coordinate conversion from cell to world."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    x, y = mapper.cell_to_world(50, 50)
    assert x == 5.0
    assert y == 5.0


def test_bresenham():
    """Test Bresenham line algorithm."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    cells = mapper.bresenham(0, 0, 5, 5)
    assert len(cells) > 0
    assert (0, 0) in cells
    assert (5, 5) in cells


def test_integrate_scan():
    """Test scan integration into map."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    pose = (1.0, 1.0, 0.0)
    ranges = np.array([1.0, 2.0, 3.0])
    angles = np.array([0.0, 0.5, -0.5])
    
    mapper.integrate_scan(pose, ranges, angles, max_range=6.0)
    
    # Check that log_odds was updated
    assert not np.all(mapper.log_odds == 0)


def test_get_prob_map():
    """Test probability map generation."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    # Add some data
    pose = (1.0, 1.0, 0.0)
    ranges = np.array([1.0, 2.0, 3.0])
    angles = np.array([0.0, 0.5, -0.5])
    mapper.integrate_scan(pose, ranges, angles, max_range=6.0)
    
    prob_map = mapper.get_prob_map()
    assert prob_map.shape == (mapper.height, mapper.width)
    assert np.all(prob_map >= 0)
    assert np.all(prob_map <= 1)


def test_get_binary_map():
    """Test binary map generation."""
    mapper = OccupancyGridMapper(size_x=10.0, size_y=10.0, resolution=0.1)
    
    # Add some data
    pose = (1.0, 1.0, 0.0)
    ranges = np.array([1.0, 2.0, 3.0])
    angles = np.array([0.0, 0.5, -0.5])
    mapper.integrate_scan(pose, ranges, angles, max_range=6.0)
    
    binary_map = mapper.get_binary_map(thresh=0.6)
    assert binary_map.shape == (mapper.height, mapper.width)
    assert np.all((binary_map == 0) | (binary_map == 1))


if __name__ == "__main__":
    test_mapper_initialization()
    test_world_to_cell()
    test_cell_to_world()
    test_bresenham()
    test_integrate_scan()
    test_get_prob_map()
    test_get_binary_map()
    print("All mapping tests passed!")
