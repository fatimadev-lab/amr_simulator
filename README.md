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

2. Install dependencies:

   pip install -r C:\Users\Administrator\amr_simulator\requirements.txt

3. Run the demo:

   python C:\Users\Administrator\amr_simulator\run.py

Notes
- The demo opens an interactive matplotlib window showing the current occupancy probability map, the true map overlay, robot pose, goal, and current planned path.
- Mapping uses odometry (with small simulated noise) as the pose input. This is the simpler odometry + occupancy-grid approach.

Project layout
- simulator.py — LiDAR and simple kinematics simulator
- mapping.py — OccupancyGridMapper (log-odds)
- planner.py — AStarPlanner for grid planning
- avoidance.py — ObstacleAvoider (reactive safety checks)
- run.py — Demo runner and matplotlib visualization
- requirements.txt — required Python packages

Limitations and next steps
- This is a proof-of-concept. It does not implement full SLAM (no pose-uncertainty handling or loop closure).
- Dynamic obstacles are handled reactively by stopping and replanning. For robust dynamic handling, consider adding a moving-object tracker and temporally-weighted occupancy or velocity estimates.

Contributing
- Improvements welcome: better sensor models, particle-filter-based SLAM (FastSLAM), scan-matching (ICP), or porting to ROS for real-robot experiments.

License
This project is released under the MIT License. See LICENSE for details.

Acknowledgements
- Created with assistance from an AI assistant (Copilot CLI runtime in VS Code).
