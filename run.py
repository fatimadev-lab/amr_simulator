"""
Run the AMR simulation: build environment, run simulator loop, map with occupancy grid, plan with A*, and avoid dynamic obstacles.
Visualize with matplotlib.
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from simulator import Simulator
from mapping import OccupancyGridMapper
from planner import AStarPlanner
from avoidance import ObstacleAvoider

# Build a simple maze-like true map as a binary grid
def make_maze_grid(w=200, h=150):
    grid = np.zeros((h, w), dtype=np.uint8)
    # border walls
    grid[0,:] = 1; grid[-1,:] = 1; grid[:,0] = 1; grid[:,-1] = 1
    # internal walls: vertical and horizontal
    grid[20:130,40] = 1
    grid[20,40:160] = 1
    grid[130,40:160] = 1
    grid[20:80,100] = 1
    grid[80:140,140] = 1
    # opening in some walls
    grid[40,40] = 0
    grid[130,120] = 0
    return grid


def main():
    resolution = 0.05  # meters per cell
    true_map = make_maze_grid(200,150)
    sim = Simulator(true_map, resolution=resolution, lidar_range=8.0, n_beams=120)

    # occupancy grid mapper world size in meters
    size_x = true_map.shape[1] * resolution
    size_y = true_map.shape[0] * resolution
    mapper = OccupancyGridMapper(size_x, size_y, resolution=resolution)

    # robot start and goal (in world coords)
    start_world = (1.0, 1.0)
    goal_world = (8.5, 6.0)
    pose = (start_world[0], start_world[1], 0.0)

    avoider = ObstacleAvoider(safety_radius=0.4)

    fig, ax = plt.subplots(figsize=(8,6))
    ims = []

    # planning variables
    plan = None
    plan_step = 0

    # animation update
    def update(frame):
        nonlocal pose, plan, plan_step
        # simulate sensor
        ranges = sim.get_lidar(pose)
        angles = sim.angles
        # integrate scan into mapper (use odometry pose)
        mapper.integrate_scan(pose, ranges, angles, max_range=sim.lidar_range)

        # check dynamic obstacle
        collision = avoider.check_immediate_collision(pose, ranges, angles)
        if collision:
            # stop and replan: mark beams closer than safety as occupied in mapper log-odds temporarily
            # For simplicity, just trigger replanning
            plan = None
            plan_step = 0

        # if no plan, compute A* on current binary map
        if plan is None:
            occ_bin = mapper.get_binary_map(thresh=0.6)
            planner = AStarPlanner(occ_bin)
            sx, sy = mapper.world_to_cell(pose[0], pose[1])
            gx, gy = mapper.world_to_cell(goal_world[0], goal_world[1])
            start_cell = (sx, sy)
            goal_cell = (gx, gy)
            p = planner.plan(start_cell, goal_cell)
            plan = p
            plan_step = 0

        # follow plan: move toward next cell
        v = 0.5
        w = 0.0
        if plan is not None and plan_step < len(plan):
            nx, ny = plan[plan_step]
            tx, ty = mapper.cell_to_world(nx, ny)
            dx = tx - pose[0]
            dy = ty - pose[1]
            desired_yaw = np.arctan2(dy, dx)
            yaw_err = (desired_yaw - pose[2] + np.pi) % (2*np.pi) - np.pi
            if abs(yaw_err) > 0.2:
                # rotate in place
                v_cmd = 0.0
                w_cmd = 1.0 * np.sign(yaw_err)
            else:
                v_cmd = 0.6
                w_cmd = 0.0
                dist = np.hypot(dx,dy)
                if dist < 0.1:
                    plan_step += 1
        else:
            v_cmd = 0.0
            w_cmd = 0.0

        # if immediate collision detected, stop
        if collision:
            v_cmd = 0.0
            w_cmd = 0.0

        # step robot (get noisy odometry pose)
        pose = sim.step_pose(pose, v_cmd, w_cmd, dt=0.1)

        # draw visualization
        ax.clear()
        probmap = mapper.get_prob_map()
        ax.imshow(probmap, cmap='gray', origin='lower', vmin=0, vmax=1)
        # draw true map overlay with transparency
        ax.imshow(sim.true_map, cmap='Reds', alpha=0.25, origin='lower')
        # draw robot
        rx, ry, ryaw = pose
        ix, iy = mapper.world_to_cell(rx, ry)
        ax.plot(ix, iy, 'bo')
        # draw goal
        gx_c, gy_c = mapper.world_to_cell(goal_world[0], goal_world[1])
        ax.plot(gx_c, gy_c, 'gx')
        # draw current plan
        if plan is not None:
            pxs = [p[0] for p in plan]
            pys = [p[1] for p in plan]
            ax.plot(pxs, pys, '-y')
        ax.set_title(f"Frame {frame}  collision={collision}")
        ax.set_xlim(0, mapper.width)
        ax.set_ylim(0, mapper.height)
        return []

    ani = animation.FuncAnimation(fig, update, frames=500, interval=100, blit=False)
    plt.show()

if __name__ == '__main__':
    main()
