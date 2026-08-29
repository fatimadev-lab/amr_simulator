"""
Occupancy grid mapping using inverse sensor model and Bresenham ray tracing.
Assumes odometry provides pose (x,y,yaw) in meters.
"""
import numpy as np

class OccupancyGridMapper:
    def __init__(self, size_x, size_y, resolution=0.1, prob_prior=0.5):
        self.resolution = resolution
        self.size_x = size_x
        self.size_y = size_y
        self.width = int(np.ceil(size_x / resolution))
        self.height = int(np.ceil(size_y / resolution))
        # log-odds representation
        self.log_odds = np.zeros((self.height, self.width), dtype=float)
        self.l0 = self.log_odds.copy()  # prior (0)
        self.prior = prob_prior
        self.l_occ = np.log(0.9 / (1 - 0.9))
        self.l_free = np.log(0.3 / (1 - 0.3))

    def world_to_cell(self, x, y):
        ix = int(np.clip(round(x / self.resolution), 0, self.width - 1))
        iy = int(np.clip(round(y / self.resolution), 0, self.height - 1))
        return ix, iy

    def cell_to_world(self, ix, iy):
        x = ix * self.resolution
        y = iy * self.resolution
        return x, y

    def bresenham(self, x0, y0, x1, y1):
        # return integer cells along the line from (x0,y0) to (x1,y1) inclusive
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        cells = []
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells

    def integrate_scan(self, pose, ranges, angles, max_range):
        # pose: (x,y,yaw) in meters. ranges: distances per beam. angles: beam angles relative to robot yaw.
        rx, ry, yaw = pose
        for r, a in zip(ranges, angles):
            theta = yaw + a
            # endpoint in world coords
            ex = rx + r * np.cos(theta)
            ey = ry + r * np.sin(theta)
            ix0, iy0 = self.world_to_cell(rx, ry)
            ix1, iy1 = self.world_to_cell(ex, ey)
            cells = self.bresenham(ix0, iy0, ix1, iy1)
            # mark free for all except endpoint
            if r < max_range - 1e-6:
                for (cx, cy) in cells[:-1]:
                    self.log_odds[cy, cx] += self.l_free
                # endpoint occupied
                ex_cx, ex_cy = cells[-1]
                self.log_odds[ex_cy, ex_cx] += self.l_occ
            else:
                for (cx, cy) in cells:
                    self.log_odds[cy, cx] += self.l_free

    def get_prob_map(self):
        odds = np.exp(self.log_odds)
        return odds / (1 + odds)

    def get_binary_map(self, thresh=0.6):
        return (self.get_prob_map() >= thresh).astype(np.uint8)


if __name__ == "__main__":
    # small self test
    m = OccupancyGridMapper(10, 10, resolution=0.1)
    pose = (1.0, 1.0, 0.0)
    ranges = np.array([1.0, 2.0, 3.0])
    angles = np.array([0.0, 0.5, -0.5])
    m.integrate_scan(pose, ranges, angles, max_range=6.0)
    print(m.get_prob_map().shape)
