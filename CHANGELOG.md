# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-08-29

### Added
- Initial release of AMR Simulator
- LiDAR raycasting simulator against a ground-truth binary occupancy map
- Occupancy grid mapping using log-odds and Bresenham ray tracing
- A* path planner over the binary occupancy map
- Simple reactive obstacle avoidance
- Top-level demo with matplotlib visualization
- Comprehensive test suite covering all modules
- Setup.py for package installation
- Contributing guidelines
- This changelog

### Features
- Simulated 2D environment with wall obstacles
- Realistic LiDAR scanning with configurable beam count and range
- Odometry with simulated noise
- Inverse sensor model for occupancy grid updates
- Efficient A* pathfinding with Euclidean heuristic
- Reactive safety checks for dynamic obstacle avoidance
- Real-time matplotlib visualization

### Known Limitations
- Does not implement full SLAM (no pose-uncertainty handling or loop closure)
- Dynamic obstacles handled reactively only; no moving-object tracking
- Simplified odometry model without Kalman filtering
- Limited to 2D environments

## Future Roadmap

### Planned Features
- [ ] Particle filter-based SLAM (FastSLAM)
- [ ] Scan matching with ICP (Iterative Closest Point)
- [ ] ROS node integration for real-robot experiments
- [ ] 3D environment simulation support
- [ ] Advanced path planning (RRT*, PRM)
- [ ] Dynamic obstacle tracking and prediction
- [ ] Sensor noise models (Gaussian blur, drop-outs)
- [ ] Performance optimization for large maps

### Under Consideration
- Multi-robot simulation
- Costmap-based navigation
- Behavior trees for higher-level planning
- Integration with Gazebo simulator
