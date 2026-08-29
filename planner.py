"""
A* path planner on the occupancy grid.
"""
import heapq
import numpy as np

MOVES = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
MOVE_COST = [1.0,1.0,1.0,1.0,1.4,1.4,1.4,1.4]

class AStarPlanner:
    def __init__(self, occupancy_binary):
        # occupancy_binary: 2D numpy array where 1 is obstacle, 0 free
        self.occ = occupancy_binary
        self.h = self.occ.shape[0]
        self.w = self.occ.shape[1]

    def in_bounds(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def is_free(self, x, y):
        return self.in_bounds(x, y) and self.occ[y, x] == 0

    def heuristic(self, a, b):
        # Euclidean
        return np.hypot(b[0]-a[0], b[1]-a[1])

    def plan(self, start, goal):
        # start, goal are (ix, iy) in cell coordinates
        sx, sy = start
        gx, gy = goal
        if not self.is_free(gx, gy):
            return None
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, None))
        came_from = {}
        gscore = {start: 0}
        while open_set:
            f, g, current, parent = heapq.heappop(open_set)
            if current in came_from:
                continue
            came_from[current] = parent
            if current == goal:
                # reconstruct path
                path = []
                cur = current
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                path.reverse()
                return path
            cx, cy = current
            for (dx, dy), cost in zip(MOVES, MOVE_COST):
                nx = cx + dx
                ny = cy + dy
                if not self.is_free(nx, ny):
                    continue
                tentative_g = g + cost
                ncoord = (nx, ny)
                if ncoord in gscore and tentative_g >= gscore[ncoord]:
                    continue
                gscore[ncoord] = tentative_g
                heapq.heappush(open_set, (tentative_g + self.heuristic(ncoord, goal), tentative_g, ncoord, current))
        return None

if __name__ == "__main__":
    import numpy as np
    m = np.zeros((100,100), dtype=np.uint8)
    m[50,20:80] = 1
    planner = AStarPlanner(m)
    path = planner.plan((10,10),(90,90))
    print("path len", 0 if path is None else len(path))
