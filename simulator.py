"""
Simple 2D maze simulator and LiDAR simulator.
Produces range scans against a true occupancy map (walls) and returns noisy odometry.
"""
import numpy as np

class Simulator:
    def __init__(self, true_map, resolution=0.1, lidar_range=6.0, n_beams=108, odom_noise=0.01):
        # true_map: 2D numpy array: 1 = occupied, 0 = free (cells)
        self.true_map = true_map
        self.resolution = resolution
        self.lidar_range = lidar_range
        self.n_beams = n_beams
        self.angles = np.linspace(-np.pi, np.pi, n_beams, endpoint=False)
        self.odom_noise = odom_noise
        self.height, self.width = true_map.shape

    def world_to_map(self, x, y):
        # Convert world meters to map cell indices
        ix = int(np.clip(round(x / self.resolution), 0, self.width - 1))
        iy = int(np.clip(round(y / self.resolution), 0, self.height - 1))
        return ix, iy

    def map_to_world(self, ix, iy):
        x = ix * self.resolution
        y = iy * self.resolution
        return x, y

    def raycast(self, x, y, theta):
        # Simple raycast on grid using DDA, returns distance to nearest occupied cell up to lidar_range
        max_dist = self.lidar_range
        steps = int(max_dist / self.resolution)
        dx = np.cos(theta) * self.resolution
        dy = np.sin(theta) * self.resolution
        px, py = x, y
        for i in range(steps):
            px += dx
            py += dy
            ix, iy = self.world_to_map(px, py)
            if self.true_map[iy, ix]:
                dist = np.hypot(px - x, py - y)
                return min(dist, max_dist)
        return max_dist

    def get_lidar(self, pose):
        x, y, yaw = pose
        ranges = np.zeros(self.n_beams, dtype=float)
        for i, a in enumerate(self.angles):
            theta = yaw + a
            ranges[i] = self.raycast(x, y, theta)
        return ranges

    def step_pose(self, pose, v, w, dt=0.1):
        # Differential-drive kinematics simple integrator
        x, y, yaw = pose
        if abs(w) < 1e-6:
            x += v * np.cos(yaw) * dt
            y += v * np.sin(yaw) * dt
        else:
            x += (v / w) * (np.sin(yaw + w * dt) - np.sin(yaw))
            y += (v / w) * (-np.cos(yaw + w * dt) + np.cos(yaw))
            yaw += w * dt
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        # odometry noise
        x += np.random.randn() * self.odom_noise
        y += np.random.randn() * self.odom_noise
        yaw += np.random.randn() * self.odom_noise * 0.1
        return (x, y, yaw)


if __name__ == "__main__":
    # small self-test
    m = np.zeros((200, 200), dtype=np.uint8)
    m[0, :] = 1; m[-1, :] = 1; m[:, 0] = 1; m[:, -1] = 1
    sim = Simulator(m)
    pose = (1.0, 1.0, 0.0)
    ranges = sim.get_lidar(pose)
    print("sample ranges", ranges[:10])
