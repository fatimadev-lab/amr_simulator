"""
AMR Simulator - Autonomous Mobile Robot simulator with LiDAR, mapping, planning, and obstacle avoidance.

Main modules:
- simulator: LiDAR and kinematics simulation
- mapping: Occupancy grid mapping with log-odds
- planner: A* path planning
- avoidance: Reactive obstacle avoidance
"""

__version__ = "0.1.0"
__author__ = "Administrator"
__license__ = "MIT"

from simulator import Simulator
from mapping import OccupancyGridMapper
from planner import AStarPlanner
from avoidance import ObstacleAvoider

__all__ = [
    "Simulator",
    "OccupancyGridMapper",
    "AStarPlanner",
    "ObstacleAvoider",
]
