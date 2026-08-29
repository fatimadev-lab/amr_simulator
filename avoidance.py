"""
Very simple reactive obstacle avoidance:
- If obstacle appears within safety_radius along the immediate path segment, stop and replan.
- Provides an interface to check for dynamic obstacles from current lidar ranges and the occupancy grid.
"""
import numpy as np

class ObstacleAvoider:
    def __init__(self, safety_radius=0.5):
        self.safety_radius = safety_radius

    def check_immediate_collision(self, pose, ranges, angles):
        # pose: (x,y,yaw); ranges: array; angles: array
        # returns True if obstacle closer than safety_radius in front cone
        x,y,yaw = pose
        cone = np.cos(angles) > 0.5  # ~60 degree front cone
        close = ranges[cone] < self.safety_radius
        return np.any(close)

    def find_blocking_beams(self, ranges, angles, threshold):
        return np.where(ranges < threshold)[0]

    # High-level: if collision, user of this module should call planner to replan using updated occupancy

if __name__ == "__main__":
    oa = ObstacleAvoider(0.6)
    rng = np.array([1.0, 0.4, 1.2, 0.2])
    ang = np.array([-0.5, -0.1, 0.1, 0.6])
    print(oa.check_immediate_collision((0,0,0), rng, ang))
