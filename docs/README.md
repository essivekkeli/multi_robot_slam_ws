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





