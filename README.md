# AMR Simulator

A simple Autonomous Mobile Robot (AMR) simulator and demo written in Python.

This project demonstrates a minimal pipeline for: simulated LiDAR sensing, occupancy-grid mapping (log-odds), A* path planning, and a simple reactive obstacle avoidance loop. It's intended as a compact, self-contained prototype for learning SLAM basics, sensor processing, and pathfinding.

Features
- LiDAR raycasting simulator against a ground-truth binary occupancy map (simulator.py)
- Occupancy grid mapping using log-odds and Bresenham ray tracing (mapping.py)
- A* path planner over the binary occupancy map (planner.py)
- Simple reactive obstacle avoidance (avoidance.py)
- Top-level demo and visualization using matplotlib (run.py)

Quickstart
1. Create a Python virtual environment (recommended):

   python -m venv .venv
   .\.venv\Scripts\Activate.ps1   # PowerShell
   .\.venv\Scripts\activate.bat   # cmd.exe

2. Install the package and dependencies:

   Option A (development):
   pip install -e .

   Option B (minimal):
   pip install -r requirements.txt

3. Run the demo:

   python run.py

Testing
Run the unit tests to verify everything is working:

   pytest

Notes
- The demo opens an interactive matplotlib window showing the current occupancy probability map, the true map overlay, robot pose, goal, and current planned path.
- Mapping uses odometry (with small simulated noise) as the pose input. This is the simpler odometry + occupancy-grid approach.

Project layout
- simulator.py — LiDAR and simple kinematics simulator
- mapping.py — OccupancyGridMapper (log-odds)
- planner.py — AStarPlanner for grid planning
- avoidance.py — ObstacleAvoider (reactive safety checks)
- run.py — Demo runner and matplotlib visualization
- tests/ — Unit test suite for all modules
- setup.py — Package installation configuration
- requirements.txt — Required Python packages
- CONTRIBUTING.md — Guidelines for contributing to the project
- CHANGELOG.md — Version history and roadmap

Limitations and next steps
- This is a proof-of-concept. It does not implement full SLAM (no pose-uncertainty handling or loop closure).
- Dynamic obstacles are handled reactively by stopping and replanning. For robust dynamic handling, consider adding a moving-object tracker and temporally-weighted occupancy or velocity estimates.

Contributing
Improvements welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on:
- Setting up a development environment
- Running tests
- Coding standards
- Submitting pull requests

Areas for contribution include:
- Better sensor models
- Particle-filter-based SLAM (FastSLAM)
- Scan-matching algorithms (ICP)
- ROS integration for real-robot experiments
- Performance optimization

License
This project is released under the MIT License. See LICENSE for details.


