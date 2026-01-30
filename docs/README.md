# Distributed GLIM: Multi-Robot Probabilistic Mapping

A research framework for distributed multi-robot SLAM using GLIM (Gaussian Latent Inference Map), enabling collaborative dense probabilistic mapping with uncertainty quantification.

![System Status](https://img.shields.io/badge/status-research-orange)
![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green)

## 🎓 Research Motivation

### Problem Statement

**Simultaneous Localization and Mapping (SLAM)** enables robots to build spatial models of unknown environments. Recent deep-learning-based SLAM systems such as **GLIM (Gaussian Latent Inference Map)** provide **dense probabilistic maps** that capture both structure and uncertainty.

However, **multi-robot mapping using GLIM remains unexplored**.

### Research Gap

Existing collaborative mapping frameworks rely on:
- **Graph-SLAM** (exchanging pose graphs and loop closures)
- **ORB-SLAM** (exchanging feature graphs)
- **Occupancy grids** (discrete probabilistic representations)

These approaches **do not exchange latent probabilistic maps** with continuous uncertainty distributions.

### Research Contribution

**Developing a distributed GLIM mapping framework** could significantly improve:
- **Efficiency**: Probabilistic map fusion vs. feature matching
- **Robustness**: Uncertainty-aware map merging
- **Scalability**: Distributed inference vs. centralized optimization
- **Map Quality**: Dense representations with Gaussian uncertainty

## 🎯 Research Objectives

### Primary Objectives
1. **Implement distributed GLIM** for multi-robot systems
2. **Develop probabilistic map merging** algorithms
3. **Enable latent space exchange** between robots
4. **Quantify uncertainty** in merged maps

### Secondary Objectives
- Compare performance vs. traditional multi-robot SLAM (SLAM Toolbox, Cartographer)
- Evaluate communication efficiency (bandwidth, latency)
- Benchmark map quality metrics (accuracy, completeness, consistency)
- Test scalability (number of robots, environment size)

## 📊 Current Implementation Status

### Phase 1: Baseline System ✅ **COMPLETE**

**Purpose**: Establish baseline multi-robot SLAM infrastructure

**Implementation**:
- 3-robot differential drive system in Gazebo
- SLAM Toolbox (traditional feature-based SLAM) per robot
- Basic occupancy grid merging (overlay method)
- ROS 2 Jazzy + Gazebo Harmonic infrastructure

**Why This Baseline**:
- Validate multi-robot infrastructure before GLIM integration
- Establish performance benchmarks for comparison
- Debug system issues (namespacing, transforms, communication)
- Proof-of-concept for distributed mapping architecture

### Phase 2: GLIM Integration ⏳ **NEXT**

**Goal**: Replace SLAM Toolbox with GLIM for single robot

**Research Questions**:
- How does GLIM perform vs. SLAM Toolbox in simulation?
- What are the computational requirements?
- How to extract probabilistic maps from GLIM?

### Phase 3: Distributed GLIM Framework ⏳ **CORE RESEARCH**

**Goal**: Enable multi-robot GLIM with latent map exchange

**Key Innovations**:
- Probabilistic map representation protocol
- Distributed Gaussian inference algorithm
- Uncertainty-aware map fusion
- Communication-efficient latent space encoding

### Phase 4: Evaluation & Validation ⏳

**Goal**: Comprehensive benchmarking and comparison

**Metrics**:
- Map accuracy (vs. ground truth)
- Completeness (coverage percentage)
- Consistency (overlap agreement)
- Computational cost (CPU, memory)
- Communication overhead (bandwidth)
- Scalability (time vs. robots)

## 🏗️ System Architecture
```
┌─────────────────────────────────────────────────────────┐
│              Gazebo Simulation Environment               │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │ Robot 1  │    │ Robot 2  │    │ Robot 3  │          │
│  │ (LiDAR)  │    │ (LiDAR)  │    │ (LiDAR)  │          │
│  └─────┬────┘    └─────┬────┘    └─────┬────┘          │
└────────┼───────────────┼───────────────┼────────────────┘
         │               │               │
    ┌────┴───────────────┴───────────────┴─────┐
    │         ROS 2 Communication Layer         │
    └────┬───────────────┬───────────────┬──────┘
         │               │               │
    ┌────▼────┐     ┌────▼────┐     ┌───▼─────┐
    │ GLIM    │     │ GLIM    │     │ GLIM    │
    │ Node 1  │     │ Node 2  │     │ Node 3  │
    │(Latent) │     │(Latent) │     │(Latent) │
    └────┬────┘     └────┬────┘     └───┬─────┘
         │               │               │
         └───────────────┼───────────────┘
                         │
                ┌────────▼────────┐
                │ Distributed     │
                │ Map Fusion      │
                │ (Probabilistic) │
                └────────┬────────┘
                         │
                    ┌────▼─────┐
                    │ Global   │
                    │ Gaussian │
                    │ Map      │
                    └──────────┘
```

### Current Architecture (Phase 1 - Baseline)
- **Frontend**: SLAM Toolbox (feature-based)
- **Map Format**: Occupancy grids (discrete)
- **Merging**: Simple overlay (no uncertainty)
- **Communication**: Full maps exchanged

### Target Architecture (Phase 3 - Research Goal)
- **Frontend**: GLIM (Gaussian latent)
- **Map Format**: Continuous Gaussian fields
- **Merging**: Probabilistic fusion (KL divergence, Bayesian)
- **Communication**: Latent representations (compressed)

## 📚 Related Work

### Multi-Robot SLAM Systems

**Graph-SLAM Based**:
- CCM-SLAM (Schmuck & Chli, 2019)
- DGS-SLAM (Zou et al., 2021)
- Approach: Exchange pose graphs, perform distributed optimization

**Feature-Based**:
- Multi-Robot ORB-SLAM (Labbe & Michaud, 2019)
- Approach: Share ORB features, match for loop closure

**Occupancy Grid**:
- Multirobot Map Merge (Hörner & Schulz, 2016)
- Approach: Grid alignment via ICP or feature matching

**Limitations**:
- ❌ No probabilistic uncertainty representation
- ❌ Discrete representations lose information
- ❌ High communication overhead (full graphs/grids)

### GLIM (Single Robot)

**GLIM** (Koide et al., 2023):
- Deep learning-based SLAM
- Gaussian latent representation
- Dense probabilistic maps
- Continuous uncertainty quantification

**Advantages for Multi-Robot**:
- ✅ Compact latent representations
- ✅ Probabilistic fusion via Gaussian operations
- ✅ Differentiable (gradient-based optimization)
- ✅ Uncertainty-aware merging

**Gap**: No existing multi-robot GLIM framework

## 🔬 Research Methodology

### Experimental Design

#### 1. Baseline Comparison
- **Control**: SLAM Toolbox multi-robot system (current)
- **Experimental**: GLIM multi-robot system (proposed)
- **Variables**: Number of robots, environment complexity, communication constraints

#### 2. Evaluation Metrics
- **Map Quality**:
  - Absolute Trajectory Error (ATE)
  - Relative Pose Error (RPE)
  - Map entropy (uncertainty)
  - Overlap consistency
- **Performance**:
  - CPU usage per robot
  - Memory footprint
  - Real-time factor
- **Communication**:
  - Bandwidth (bytes/second)
  - Message frequency
  - Latency

#### 3. Test Environments
- **Simulation**: Gazebo with varying complexity
- **Scenarios**:
  - Structured (corridors, rooms)
  - Unstructured (outdoor, cluttered)
  - Dynamic obstacles
- **(Future) Real Robots**: TurtleBot3 or similar

### Data Collection

- ROS 2 bag recordings of all topics
- Ground truth from Gazebo (for simulation)
- Performance profiling (CPU, RAM, network)
- Map snapshots at intervals

### Analysis

- Statistical comparison (t-tests, ANOVA)
- Visualization (map overlays, error heatmaps)
- Publication-quality plots

## 📖 Documentation Structure

### For Researchers
- [RESEARCH_PLAN.md](RESEARCH_PLAN.md) - Detailed research plan *(to be created)*
- [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md) - Related work summary *(to be created)*
- [EXPERIMENTS.md](EXPERIMENTS.md) - Experimental setup and results *(to be created)*
- [PUBLICATIONS.md](PUBLICATIONS.md) - Papers and presentations *(to be created)*

### For Developers
- [DOCUMENTATION.md](DOCUMENTATION.md) - Technical documentation
- [CHANGELOG.md](CHANGELOG.md) - Version history
- [TODO.md](TODO.md) - Development roadmap

### For Users
- [README.md](README.md) - This file (quick start)
- [INSTALLATION.md](INSTALLATION.md) - Setup instructions *(to be created)*
- [USAGE.md](USAGE.md) - How to run experiments *(to be created)*

## 🚀 Quick Start (Current Baseline System)

### Installation
```bash
# Install dependencies
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-ros-gz-bridge

# Build workspace
cd ~/multi_robot_slam_ws
colcon build
source install/setup.bash
```

### Run Baseline System
```bash
ros2 launch multi_robot_slam full_system.launch.py
```

### Run Experiments
```bash
# Record data
ros2 bag record -a -o experiment_1

# Move robots (manual or automated)
ros2 run multi_robot_slam robot_controller.py

# Analyze results
python3 scripts/analyze_maps.py experiment_1
```

## 📊 Preliminary Results

### Baseline System (SLAM Toolbox)

| Metric | Value | Notes |
|--------|-------|-------|
| Map Update Rate | ~1 Hz | Per robot |
| CPU Usage | 50-70% | 3 robots |
| Memory | 4-6 GB | Full system |
| Communication | ~500 KB/s | Full maps |

**Observations**:
- Simple overlay merging causes misalignment
- No uncertainty quantification in merged map
- High communication overhead

*(GLIM results to be added after Phase 2)*

## 🎯 Research Milestones

| Milestone | Target | Status |
|-----------|--------|--------|
| M1: Baseline System | Q1 2026 | ✅ Complete |
| M2: Single-Robot GLIM | Q2 2026 | ⏳ Planned |
| M3: Multi-Robot GLIM Protocol | Q2 2026 | ⏳ Planned |
| M4: Distributed Inference | Q3 2026 | ⏳ Planned |
| M5: Full Evaluation | Q3 2026 | ⏳ Planned |
| M6: Paper Submission | Q4 2026 | 🎯 Goal |

## 📝 Publications & Presentations

### Planned Publications
- [ ] Conference paper: "Distributed GLIM: Multi-Robot Probabilistic Mapping with Gaussian Latent Inference"
  - Target: ICRA 2027 or IROS 2027
- [ ] Journal paper: Extended evaluation and analysis
  - Target: IEEE T-RO or Autonomous Robots

### Presentations
- [ ] Lab presentation: Baseline system demo
- [ ] Conference poster/demo

## 🤝 Collaboration & Contact

**Primary Researcher**: [Your Name]  
**Institution**: [Your University/Lab]  
**Advisor**: [Advisor Name]  
**Email**: [your.email@university.edu]

**Collaboration Opportunities**:
- GLIM integration and optimization
- Distributed inference algorithms
- Real-world robot experiments
- Comparative SLAM benchmarking

## 📄 License & Citation

### License
Apache License 2.0 (code)  
CC BY 4.0 (documentation)

### Citation
If you use this work, please cite:
```bibtex
@misc{yourname2026distributedglim,
  author = {Your Name},
  title = {Distributed GLIM: Multi-Robot Probabilistic Mapping},
  year = {2026},
  howpublished = {\url{https://github.com/yourusername/multi_robot_slam}},
  note = {Research in progress}
}
```

## 🙏 Acknowledgments

- GLIM authors (Koide et al.)
- SLAM Toolbox (Steve Macenski)
- ROS 2 & Gazebo communities
- [Your lab/institution]
- [Funding sources]

---

**Project Type**: Research  
**Status**: Active Development (Phase 1 Complete)  
**Version**: 0.1.0 (Baseline)  
**Last Updated**: 2026-01-29