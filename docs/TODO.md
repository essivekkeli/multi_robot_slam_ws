# TODO List

Distributed GLIM Multi-Robot Mapping Framework

**Research Goal**: Develop novel distributed probabilistic mapping using GLIM for multi-robot systems


System development roadmap and task tracking.

**Legend**: 
- ✅ Complete
- 🔄 In Progress
- ⏳ Planned
- 🔴 Blocked
- ⭐ High Priority
- 🎯 Goal/Milestone

Last Updated: 2026-01-29

---

## 🎯 Milestone 1: Baseline Infrastructure (v0.1.0) ✅

**Target**: Establish functional multi-robot SLAM system  implementing Team SLAM as a first step since it's relatively siple multi-robot implementation
**Status**: COMPLETE (2026-01-29)

### Tasks
- [x] Set up ROS 2 Jazzy workspace
- [x] Create Gazebo simulation environment
- [x] Design robot URDF with LiDAR sensor
- [x] Implement robot spawning with namespaces
- [x] Configure SLAM Toolbox for multiple robots
- [x] Create central server for map merging
- [x] Set up ROS-Gazebo bridges
- [x] Configure RViz visualization
- [x] Write comprehensive documentation
- [x] Fix launch file scoping issues
- [x] Test full system integration

### Deliverables
- ✅ Working simulation with 3 robots
- ✅ Individual SLAM per robot
- ✅ Basic map merging
- ✅ RViz visualization
- ✅ Documentation (README, CHANGELOG, TODO)

---

## 🎯 Milestone 2: Visualization & Stability (v0.2.0) ⏳

**Target**: Robust visualization and improved mapping  
**Priority**: ⭐ HIGH  
**Status**: Not Started

### Phase 2.1: RViz Improvements ⏳

#### Tasks
- [ ] ⭐ Fix Fixed Frame issue automatically
  - [ ] Add global `map` frame publisher in central server
  - [ ] Publish TF transforms from global map to robot maps
  - [ ] Update RViz config to use global `map` frame
  - [ ] Test frame transformations with all robots
- [ ] Add robot model visualization (mesh/URDF display)
- [ ] Create saved camera views (per-robot + global)
- [ ] Add map quality metrics display panel
- [ ] Improve color schemes for better visibility
- [ ] Add toggle buttons for individual robot data

**Acceptance Criteria**:
- RViz works without manual frame configuration
- All robot models visible in 3D
- Maps overlay correctly without misalignment

---

### Phase 2.2: Map Merging Improvements ⏳

#### Tasks
- [ ] ⭐ Implement coordinate transformation in map merging
  - [ ] Track robot poses relative to global frame
  - [ ] Transform robot-local maps to global coordinates
  - [ ] Implement proper occupancy grid fusion algorithm
  - [ ] Add configurable merge strategy (max, average, Bayesian)
- [ ] Research and evaluate `multirobot_map_merge` package
  - [ ] Install and test package
  - [ ] Compare with custom implementation
  - [ ] Integration decision
- [ ] Add loop closure detection between robots
- [ ] Implement map alignment algorithms (ICP, feature matching)
- [ ] Add merge quality metrics (overlap percentage, alignment error)

**Research Needed**:
- Map registration algorithms (ICP, NDT)
- Occupancy grid fusion methods
- Multi-robot rendezvous detection

**Acceptance Criteria**:
- Maps align correctly regardless of robot starting positions
- Overlapping areas merge intelligently
- Global map shows consistent representation

---

### Phase 2.3: System Stability ⏳

#### Tasks
- [ ] Add node health monitoring
- [ ] Implement automatic restart for crashed nodes
- [ ] Add logging levels (debug, info, warn, error)
- [ ] Create system diagnostics publisher
- [ ] Add parameter validation on startup
- [ ] Improve error messages and user feedback

**Acceptance Criteria**:
- System runs for >30 minutes without crashes
- Clear error messages for common issues
- Automatic recovery from sensor/node failures

---

## 🎯 Milestone 3: Autonomous Navigation (v0.3.0) ⏳

**Target**: Nav2 integration with multi-robot support  
**Priority**: ⭐ HIGH  
**Status**: Not Started

### Phase 3.1: Single Robot Navigation ⏳

#### Tasks
- [ ] ⭐ Install Nav2 packages
```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```
- [ ] Create `nav2_params.yaml` configuration
  - [ ] Configure global costmap
  - [ ] Configure local costmap
  - [ ] Configure planners (NavFn, SMAC)
  - [ ] Configure controllers (DWB, TEB)
  - [ ] Configure behavior trees
- [ ] Create `single_robot_nav.launch.py`
- [ ] Test navigation with robot1 only
  - [ ] Goal setting via RViz
  - [ ] Goal setting via CLI
  - [ ] Goal setting via Python API
- [ ] Tune parameters for smooth navigation
- [ ] Test obstacle avoidance
- [ ] Test dynamic replanning

**Learning Resources**:
- Nav2 official tutorials
- Nav2 parameter tuning guide
- Behavior tree documentation

**Acceptance Criteria**:
- Robot1 autonomously navigates to goals
- Avoids static obstacles (walls)
- Replans when path is blocked

---

### Phase 3.2: Multi-Robot Navigation ⏳

#### Tasks
- [ ] Scale Nav2 to 3 robots (separate namespaces)
- [ ] Implement collision avoidance between robots
  - [ ] Inflated robot footprints in costmaps
  - [ ] Priority-based coordination
  - [ ] Velocity obstacles algorithm (optional)
- [ ] Create multi-robot launch file
- [ ] Test 3 robots navigating simultaneously
- [ ] Handle navigation conflicts/deadlocks

**Acceptance Criteria**:
- All 3 robots navigate autonomously
- Robots avoid each other
- No deadlock situations

---

### Phase 3.3: Robot Coordination ⏳

#### Tasks
- [ ] Implement task allocation algorithm
  - [ ] Centralized (central server assigns goals)
  - [ ] Or distributed (robots negotiate)
- [ ] Create exploration strategy
  - [ ] Frontier detection
  - [ ] Area partitioning
  - [ ] Coverage path planning
- [ ] Add multi-robot rendezvous capability
- [ ] Implement formation control (optional)

**Research Needed**:
- Multi-robot task allocation (auction-based, optimization)
- Frontier-based exploration
- Coverage planning algorithms

**Acceptance Criteria**:
- Robots autonomously explore environment
- No duplicate coverage of areas
- Efficient map building

---

## 🎯 Milestone 4: Enhanced Sensors (v0.4.0) ⏳

**Target**: Multi-sensor fusion for robustness  
**Priority**: MEDIUM    
**Status**: Not Started

### Phase 4.1: IMU Integration ⏳

#### Tasks
- [ ] Add IMU sensor to `robot.urdf.xacro`
```xml
  <sensor name="imu" type="imu">
    <topic>imu</topic>
    <update_rate>100</update_rate>
  </sensor>
```
- [ ] Create IMU-ROS bridge
- [ ] Install `robot_localization` package
- [ ] Configure Extended Kalman Filter (EKF)
  - [ ] Fuse odometry + IMU
  - [ ] Tune covariances
- [ ] Test improved pose estimation accuracy
- [ ] Update SLAM to use fused odometry

**Acceptance Criteria**:
- IMU data publishing at 100Hz
- Improved odometry accuracy (especially rotation)
- Better map quality on turns

---

### Phase 4.2: Camera Integration ⏳

#### Tasks
- [ ] Choose camera type (RGB vs Depth vs RGBD)
- [ ] Add camera to robot URDF
- [ ] Create camera-ROS bridge
- [ ] Install visual odometry package (RTAB-Map or ORB-SLAM3)
- [ ] Implement visual loop closure
- [ ] Add camera feed to RViz
- [ ] (Optional) Add object detection capability

**Research Needed**:
- Visual SLAM packages for ROS 2
- Camera calibration procedures
- Visual loop closure algorithms

**Acceptance Criteria**:
- Camera publishes images to ROS
- Visual loop closure improves mapping
- Camera data visible in RViz

---

### Phase 4.3: Additional Sensors (Optional) ⏳

#### Tasks
- [ ] Add second LiDAR for rear coverage
- [ ] Or upgrade to 3D LiDAR (Velodyne, Ouster)
- [ ] Add ultrasonic sensors for close-range detection
- [ ] Add bumper sensors for collision detection
- [ ] Implement sensor fusion for all sources

**Acceptance Criteria**:
- All sensors integrated and publishing
- Sensor fusion improves reliability
- Better obstacle detection coverage

---

## 🎯 Milestone 5: Advanced Mapping with GLIM (v0.5.0) ⏳

**Target**: Migration to GLIM for 3D mapping  
**Priority**: MEDIUM  
**Status**: Not Started  
**Prerequisites**: Enhanced sensors (Phase 4) recommended

### Phase 5.1: GLIM Setup ⏳

#### Tasks
- [ ] Deep dive into GLIM paper and documentation
  - [ ] Understand Gaussian latent representation
  - [ ] Study inference algorithm
  - [ ] Review network architecture
- [ ] Install GLIM dependencies
```bash
  sudo apt install ros-jazzy-pcl-ros
  # Additional dependencies per GLIM docs
```
- [ ] Clone and build GLIM from source
```bash
  cd ~/glim_ws/src
  git clone https://github.com/koide3/glim
```
- [ ] Test GLIM with sample datasets
- [ ] Understand GLIM configuration parameters

**Learning Resources**:
- [GLIM Repository](https://github.com/koide3/glim)
- [GLIM Paper](https://arxiv.org/abs/2306.10627)
- GLIM documentation and tutorials

---




### Phase 5.2: Single Robot GLIM Integration ⏳

**Research Questions**:
1. How does GLIM perform in our simulation vs. SLAM Toolbox?
2. What are computational requirements for real-time operation?
3. How to extract/represent probabilistic maps from GLIM?
4. What is the latent space structure?

#### Tasks
- [ ] Configure GLIM for single robot
- [ ] Replace SLAM Toolbox with GLIM in launch file
- [ ] Update topic remappings
- [ ] Test GLIM mapping with robot1
- [ ] Compare results with SLAM Toolbox
- [ ] Tune GLIM parameters
- [ ] Replace SLAM Toolbox with GLIM for robot1
- [ ] Test and debug single-robot operation
- [ ] Collect performance metrics
- [ ] Document migration process




**Acceptance Criteria**:
- GLIM produces high-quality maps
- Performance is acceptable (CPU/RAM)
- 3D mapping works (if using 3D LiDAR)

---



#### Phase 5.3: GLIM Evaluation & Analysis

**Tasks**:
- [ ] Benchmark GLIM vs. SLAM Toolbox
  - [ ] Map quality (ATE, RPE)
  - [ ] Computational cost (CPU, RAM)
  - [ ] Real-time performance
  - [ ] Map uncertainty quantification
- [ ] Analyze latent space properties
  - [ ] Dimensionality
  - [ ] Gaussian parameters (mean, covariance)
  - [ ] Compression ratio
- [ ] Document findings
- [ ] Create visualizations (plots, maps)

**Deliverables**:
- Comparative performance report
- Latent space analysis
- Decision: Proceed to multi-robot or iterate

**Decision Point**: Is GLIM suitable for multi-robot extension?




### Phase 5.4: Multi-Robot GLIM Protocol⏳

**Priority**: ⭐⭐⭐ CRITICAL (Novel Contribution)  
**Status**: Not Started  

**Research Questions**:
1. How to represent/communicate Gaussian latent maps?
2. What is optimal communication protocol?
3. How to synchronize distributed GLIM nodes?
4. How to handle communication failures?


#### Phase 5.6: Latent Map Communication Protocol

**Tasks**:
- [ ] Design latent map message format
  - [ ] Define Gaussian parameters structure
  - [ ] Add metadata (timestamp, robot ID, uncertainty)
  - [ ] Optimize serialization (compression)
- [ ] Implement ROS 2 custom messages
```bash
  # Create multi_robot_glim_msgs package
  - GaussianLatentMap.msg
  - LatentMapUpdate.msg
  - MapMergeRequest.msg
```
- [ ] 📡 Create publisher/subscriber nodes
  - [ ] Latent map publisher (per robot)
  - [ ] Latent map subscriber (central server)
  - [ ] Handshake protocol
- [ ] 🧪 Test communication with 2 robots

**Research Considerations**:
- Bandwidth constraints
- Message frequency vs. map quality
- Lossy vs. lossless compression

**Deliverables**:
- Custom ROS 2 messages for latent maps
- Communication protocol documentation
- 2-robot latent exchange working

---


#### Phase 5.7 Distributed GLIM Architecture

**Tasks**:
- [ ] 🏗️ Design distributed system architecture
  - [ ] Centralized vs. decentralized fusion
  - [ ] Synchronization mechanism
  - [ ] Conflict resolution strategy
- [ ] 💻 Implement distributed GLIM manager
  - [ ] Robot registration/discovery
  - [ ] Map fusion coordinator
  - [ ] Latency compensation
- [ ] 🔄 Scale to 3 robots
- [ ] 🧪 Test various network conditions
  - [ ] Latency injection
  - [ ] Packet loss simulation
  - [ ] Bandwidth throttling

**Deliverables**:
- Distributed GLIM system design document
- Implementation for 3 robots
- Network robustness tests

---


#### Phase 5.8 Probabilistic Map Fusion Algorithm

**Tasks**:
- [ ] 📚 Literature review on Gaussian fusion methods
  - [ ] Kalman filter fusion
  - [ ] Covariance intersection
  - [ ] KL divergence minimization
  - [ ] Bayesian fusion
- [ ] 🧮 Implement fusion algorithms
  - [ ] Algorithm 1: [Method name]
  - [ ] Algorithm 2: [Method name]
  - [ ] Algorithm 3: [Method name]
- [ ] 🔬 Compare fusion methods
  - [ ] Accuracy
  - [ ] Computational cost
  - [ ] Robustness to misalignment
- [ ] ⚡ Optimize best-performing algorithm
- [ ] 🧪 Validate with ground truth

**Research Contribution**: Novel Gaussian fusion for GLIM

**Deliverables**:
- Multiple fusion algorithms implemented
- Comparative evaluation
- Optimized fusion algorithm
- Publication-quality results




---

## 🎯 Milestone 6: Advanced features & Optimization (v1.0.0) ⏳

**Target**: Production-ready system  
**Priority**: LOW (but important for release)  
**Status**: Not Started

### Phase 6.1: Loop closure for Multi-Robot GLIM  ⏳


**Tasks**:
- [ ] Implement inter-robot loop closure detection
  - [ ] Latent space similarity metric
  - [ ] Place recognition in Gaussian space
- [ ] Integrate loop closure into fusion
- [ ] Evaluate improvement in map consistency

**Research Question**: Can latent space similarity replace traditional feature matching?

---

### Phase 6.2: Scalability testing ⏳

**Tasks**:
- [ ] Test with 5, 7, 10 robots
- [ ] Analyze scalability limits
  - [ ] Computation scaling
  - [ ] Communication scaling
  - [ ] Map quality vs. robot count
- [ ] Document scaling characteristics

**Deliverable**: Scalability analysis report

---


### Phase 6.3: Testing & Documentation ⏳

#### Tasks
- [ ] Write unit tests for Python nodes
- [ ] Write integration tests
- [ ] Create automated test suite
```bash
  colcon test
```
- [ ] Performance benchmarking
- [ ] Create video demonstrations
- [ ] Write user manual
- [ ] Create developer guide
- [ ] Add code comments and docstrings
- [ ] Update all documentation

**Acceptance Criteria**:
- >80% code coverage
- All tests passing
- Complete documentation
- Demo videos available

---


### 🎯 Milestone 7: Comprehensive Evaluation (v0.5.0)

**Priority**: ⭐⭐ HIGH (Required for Publication)  
**Timeline**: Weeks 15-18 (Jun 2026)  
**Status**: Not Started

#### Phase 7.1: Experimental Setup

**Tasks**:
- [ ] 🗺️ Design test environments
  - [ ] Simple: Corridor (known)
  - [ ] Medium: Multi-room office
  - [ ] Complex: Outdoor with obstacles
  - [ ] Dynamic: Moving obstacles
- [ ] 📋 Define experiment protocols
  - [ ] Robot trajectories
  - [ ] Data collection procedures
  - [ ] Repeatability measures
- [ ] 🎯 Establish evaluation metrics
  - [ ] Map quality (ATE, RPE, entropy)
  - [ ] Performance (CPU, RAM, time)
  - [ ] Communication (bandwidth, latency)
  - [ ] Scalability (vs. robot count)

**Deliverable**: Experiment design document

---

#### Phase 7.2: Run Experiments

**Tasks**:
- [ ] 🧪 Baseline experiments (SLAM Toolbox)
  - [ ] 10 runs per environment
  - [ ] Record all data (rosbag)
  - [ ] Log ground truth
- [ ] 🧪 GLIM experiments (proposed system)
  - [ ] Same environments
  - [ ] Same trajectories
  - [ ] Same data collection
- [ ] 🧪 Ablation studies
  - [ ] Without loop closure
  - [ ] Without uncertainty fusion
  - [ ] Different communication rates

**Deliverable**: Complete experimental dataset

---

#### Phase 7.3: Data Analysis

**Tasks**:
- [ ] 📊 Process all experimental data
  - [ ] Compute metrics for all runs
  - [ ] Statistical analysis (t-tests)
  - [ ] Generate plots and tables
- [ ] 📈 Create visualizations
  - [ ] Map comparisons (overlays)
  - [ ] Error heatmaps
  - [ ] Performance graphs
  - [ ] Scaling curves
- [ ] 📝 Write results section
- [ ] 🎨 Create publication-quality figures

**Deliverable**: Complete results with statistical validation

---

### 🎯 Milestone 8: Publication & Dissemination (v1.0.0)

**Priority**: ⭐⭐⭐ CRITICAL (Research Goal)  
**Timeline**: Weeks 19-24 (Jul-Aug 2026)  
**Status**: Not Started

#### Phase 8.1: Paper Writing

**Tasks**:
- [ ] 📄 Write conference paper
  - [ ] Abstract
  - [ ] Introduction
  - [ ] Related Work
  - [ ] Methodology
  - [ ] Experiments
  - [ ] Results
  - [ ] Discussion
  - [ ] Conclusion
- [ ] 🎨 Create all figures and tables
- [ ] 📝 Write supplementary material
- [ ] 🔄 Internal review and revision





## 🔬 Research-Specific Tasks

### Literature Review (Ongoing)
- [ ] 📚 Comprehensive multi-robot SLAM survey
- [ ] 📚 GLIM-related papers (follow citations)
- [ ] 📚 Gaussian process mapping papers
- [ ] 📚 Distributed inference literature
- [ ] 📚 Communication-efficient SLAM
- [ ] 📝 Maintain annotated bibliography

### Theoretical Development
- [ ] 🧮 Derive Gaussian fusion equations
- [ ] 📐 Prove convergence properties (if possible)
- [ ] 📊 Analyze complexity (time/space)
- [ ] ✍️ Document mathematical framework

### Collaboration & Networking
- [ ] 💬 Contact GLIM authors for insights
- [ ] 👥 Engage with multi-robot SLAM community
- [ ] 🎤 Present at lab meetings (monthly)
- [ ] 📧 Reach out to potential collaborators

---


## 🔴 Blocked Items

Currently no blocked items.

---

## 💡 Ideas / Future Enhancements (Backlog)

### Long-term Ideas
- [ ] Dynamic robot addition/removal during runtime
- [ ] Support for heterogeneous robot teams (different sensors)
- [ ] Cloud-based map storage and retrieval
- [ ] Machine learning for exploration strategy
- [ ] Semantic mapping (object recognition + mapping)
- [ ] Integration with real hardware robots
- [ ] Distributed computing support (multi-machine)
- [ ] VR/AR interface for visualization
- [ ] Swarm behavior algorithms
- [ ] Energy-aware task allocation

### Research Topics
- [ ] Compare different SLAM algorithms (RTAB-Map, Cartographer, GLIM)
- [ ] Evaluate communication protocols (DDS QoS settings)
- [ ] Study multi-robot coordination algorithms
- [ ] Research place recognition methods
- [ ] Investigate graph-based SLAM approaches

---

## 📊 Progress Tracking

| Milestone | Status | Progress | Priority | Est. Duration |
|-----------|--------|----------|----------|---------------|
| M1: Core Functionality | ✅ Complete | 100% | - | 4 weeks |
| M2: Visualization & Stability | ⏳ Planned | 0% | ⭐ HIGH | 2-3 weeks |
| M3: Autonomous Navigation | ⏳ Planned | 0% | ⭐ HIGH | 3-4 weeks |
| M4: Enhanced Sensors | ⏳ Planned | 0% | MEDIUM | 3-4 weeks |
| M5: Advanced Mapping (GLIM) | ⏳ Planned | 0% | MEDIUM | 4-5 weeks |
| M6: System Polish | ⏳ Planned | 0% | LOW | 3-4 weeks |

**Total Estimated Time to v1.0.0**: ~18-24 weeks (4.5-6 months)

---

## 🎯 Current Sprint (Week of 2026-01-30)

### This Week's Goals
1. ⭐ Fix RViz Fixed Frame issue
2. ⭐ Test and validate map merging quality
3. Begin Nav2 research and planning

### Tasks
- [ ] Add global map frame publisher in central server
- [ ] Update RViz config for automatic frame handling
- [ ] Move all 3 robots and observe merged map
- [ ] Document map merging quality issues
- [ ] Install Nav2 and run tutorials
- [ ] Create nav2_params.yaml draft

---

## 📝 Notes

### Development Guidelines
- Test with single robot before scaling to three
- Commit working code frequently
- Update documentation as you go
- Profile before optimizing
- Consider real hardware constraints

### Testing Checklist (per milestone)
- [ ] Single robot functionality
- [ ] Three robot functionality
- [ ] System runs for >15 minutes without crashes
- [ ] All sensors publishing at expected rates
- [ ] RViz displays all data correctly
- [ ] Documentation updated
- [ ] CHANGELOG updated
- [ ] Git tagged with version number

---


**Last Review**: 2026-01-29  
**Next Review**: 2026-02-06  
**Maintained By**: [Essi Vekkeli]