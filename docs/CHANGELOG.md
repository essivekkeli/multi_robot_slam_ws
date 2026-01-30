# Changelog

All notable changes to the Multi-Robot SLAM System will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Planned
- Nav2 integration for autonomous navigation
- Improved map merging with coordinate transformations
- IMU sensor integration
- Camera sensors (RGB/Depth)
- Robot coordination and collision avoidance
- GLIM migration for advanced mapping

---

## [0.1.0] - 2026-01-29

### Added
- Initial multi-robot SLAM system implementation
- Gazebo Harmonic simulation environment with custom world
- Three differential drive robots with namespace isolation
- 2D LiDAR sensors (360°, 10Hz, 10m range) on each robot
- SLAM Toolbox integration for individual robot mapping
- Central server for basic map merging
- ROS-Gazebo bridges for all robot topics (cmd_vel, odom, scan, tf)
- Robot State Publishers with proper TF frame prefixes
- RViz2 configuration with multi-robot displays
- Launch files:
  - `full_system.launch.py` - Complete system launcher
  - `gz_multi_robot.launch.py` - Gazebo simulation with robots
  - `multi_robot_slam.launch.py` - SLAM nodes
  - `central_server.launch.py` - Map merging server
- Configuration files:
  - `slam_params.yaml` - SLAM Toolbox parameters
  - `multi_robot_params.yaml` - Central server configuration
  - `rviz_config.rviz` - Pre-configured visualization
- Robot URDF with xacro templating for multi-robot spawning
- Documentation:
  - Comprehensive system documentation
  - README with quick start guide
  - This CHANGELOG
  - TODO list with roadmap

### Technical Details
- **ROS 2**: Jazzy Jalisco
- **Simulator**: Gazebo Harmonic
- **SLAM Backend**: SLAM Toolbox (async mode)
- **Programming Languages**: Python 3.12, C++ (CMake)
- **Robot Specifications**:
  - Base: Cylinder (r=0.15m, h=0.1m, mass=5kg)
  - Wheels: 2x (r=0.05m, width=0.04m)
  - LiDAR: gpu_lidar, 360 samples, 0.12-10m range
- **Topics**: 
  - 3 robots × (cmd_vel, odom, scan, map, tf) = 15+ topics
  - Global map topic for merged representation
- **Update Rates**:
  - LiDAR: ~10Hz
  - Odometry: ~50Hz
  - Maps: ~1Hz
  - TF: ~50Hz

### Fixed
- Python lambda scoping issues in launch files causing "robot_name not defined" errors
  - Solution: Helper function with proper variable capture
  - Solution: String concatenation instead of f-strings in parameters
- Xacro frame parameter issues (${robot_name} vs $(arg robot_name))
- Python bytecode cache (.pyc) causing stale code execution
  - Added proper clean rebuild instructions
- RViz segmentation fault on startup
  - Created proper rviz_config.rviz file with valid YAML
- Robot spawn timing conflicts
  - Implemented staggered spawn delays (3s, 5s, 7s)
- ROS-Gazebo bridge direction syntax
  - Corrected: `[` for Gz→ROS, `@` for bidirectional

### Known Issues
- Map merging uses simple overlay without coordinate transformation
- RViz Fixed Frame must be manually set to `robot1/map` (not `map`)
- No collision avoidance between robots
- No autonomous navigation (manual control only)
- Central server lambda scoping issue in subscription callbacks (functional but not ideal)
- Gazebo libEGL warning (cosmetic, doesn't affect functionality)

### Performance
- CPU Usage: 30-40% idle, 50-70% active (3 robots + SLAM)
- RAM Usage: 4-6 GB
- System runs stably for extended periods
- All sensors publishing at expected rates

---

## [0.0.0] - Initial Planning

### Planned
- Project structure and architecture design
- Technology stack selection (ROS 2 Jazzy, Gazebo Harmonic)
- Multi-robot SLAM research and approach

---

## Version History Summary

| Version | Date | Status | Key Features |
|---------|------|--------|--------------|
| 0.1.0 | 2026-01-29 | ✅ Released | Initial multi-robot SLAM system |
| 0.2.0 | TBD | 🔄 Planned | Nav2, improved map merging |
| 0.3.0 | TBD | 📋 Future | Enhanced sensors, coordination |
| 1.0.0 | TBD | 🎯 Goal | Production-ready multi-robot SLAM |

---

## Types of Changes

- `Added` - New features
- `Changed` - Changes in existing functionality
- `Deprecated` - Soon-to-be removed features
- `Removed` - Removed features
- `Fixed` - Bug fixes
- `Security` - Vulnerability fixes

---

**Note**: This project is under active development. Breaking changes may occur before version 1.0.0.