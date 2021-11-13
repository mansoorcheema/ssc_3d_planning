

# SSC 3D Planning
**Semantic mapping and autonomous exploration planning framework for MAVs.**

![scene_completion_explroation](https://user-images.githubusercontent.com/10983181/148309878-bcc053cd-acf5-4eea-8e51-7553e92315af.gif)

A MAV constructing the SSC mapping. Cyan voxels show the scene completed map while the measured voxels are colored grey. MAV uses measuare map for collision detection while advantaging from scene completion for path sampling.


# Table of Contents
 **Documentation**
 * [Mapping](#SSC-Mapping)
 * [Planning](#SSC-Planning)
 * [Scene Completion](#SSC-Network)
 
**Getting Started**
* [**Overview**](#Overview)
 * **Dependencies**
	* [Frameworks](#Frameworks)
	* [Simulation Setup](#Simulation-Setup)

* [**Installation**](#Installation)

**Experiments**
* [Planner configurations](#Planning-Strategies)
 * [Launch Experiments](#Launch-Experiment)

**Evaluations**
* [Run experiment with evaluations](#Enable-Evaluations)
* [Metric Calculation](#Metric-Calculation)


# Documentation
## Overview
![framework_overview](https://user-images.githubusercontent.com/10983181/148326808-26d72430-cd70-4fe4-98c0-b261b3c4e77e.png)

## Description
### SSC Network
![ssc_network](https://user-images.githubusercontent.com/10983181/148430928-283bfb42-88a6-4238-921f-6d6efb68c2f8.png)

A 3D Semantic Scene Completion(SSC) Network accepts a depth map along with a pose and predicts a semantically completed volume. The completions are used by **ssc_mapping** to build a scene completed mapping. The  [**ssc_network**](https://github.com/ethz-asl/SSC) packages comprises the following components:
- PALNet based 3D SSC
- VoxelUtils  - A python library providing:
   - GPU based TSDF computation
   - 3D projection indices from a 2D depth image
- ROS Wrapper
	- Retrieve depth images and pose
	- Publish scene completed volumes	

### SSC Mapping
![ssc_mapping_small](https://user-images.githubusercontent.com/10983181/148431816-37e10156-5747-465b-bf84-4d41852aee4f.png)

3D dense volumetric map comprising a measured TSDF map and a scene completed probabilistic map. The main components are:
- **Voxblox**
- **SSCServer**
	- ROS adapter to receive scene completions from **ssc_network**
- **SSCOccupancyMap** A dense 3D probabilistic mapping representation for maintaining scene completed map.
- **Fusion Manager** contains a reference to **SSCOccupancyMap** for maintaining scene completed map and performs:
	- Voxel Fusion - Fuses a predicted voxel into an existing voxel following a fusion strategy
	- Volume Integration - Integrate a scene completed volume into the scene completed map
### SSC Planning
![ssc_planning_overview](https://user-images.githubusercontent.com/10983181/148425994-46c0ee6e-d3c0-4bb5-b390-8520374207ea.png)

Informative Path Planner utilizing measured and scene completed maps for exploration planning. It's core components are:

* **Scene Completed Mapping**  module maintains an internal reference to **ssc_mapping** for planning.
*  **Gain Evaluator** calculates informative gain for a trajectory segment by consulting the Scene Completed Map
* **Trajectory Generator** generates new trajectory segments by exploiting the scene completed mapping and forwards the segments to trajectory gain evaluator module

# Getting Started
## Overview

* **ssc_mapping**

   Dense volumetric mapping package leveraging deep learnt priors using 3D SSC Network
* **ssc_planning**

  Online Informative path planner using **ssc_mapping** as mapping representation
* **ssc_network**

   3D Semantic Scene Completion(SSC)  Network for completing partial depth scans:
    * `SSC` ([https://github.com/ethz-asl/SSC](https://github.com/ethz-asl/SSC))
 * **ssc_msgs**
 ROS message for publishing scene completions.  Publisher: **ssc_network**, Subscriber: **ssc_mapping**
## Dependencies
### Frameworks
* **voxblox**
 Volumetric mapping framework `voxblox`  ([https://github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox)) 
* **active_ planning_3d** 
	The following packages are required from `active_planning_3d`([https://github.com/ethz-asl/mav_voxblox_planning)
	* `core` 
    * `ros`
   * `app_reconstruction`
* **ssc_network**
  Setup semantic scene completion framework `SSC` ([https://github.com/ethz-asl/SSC](https://github.com/ethz-asl/SSC))

### Simulation Setup
* **Unreal Engine**
Setup Unreal Engine 4.25.6 
* **Airsim**
 Airsim is a simulation software for simulating a MAV in Unreal Engine. We use Airsim 1.2 plugin for the Unreal Engine.
* **unreal_airsim** (https://github.com/ethz-asl/unreal_airsim)
 ROS  interface to the simulated MAV in Unreal Engine for accessing odometry and sending trajectory commands. This project was tested on a modified `unreal_airsim`including a PD Controller  available at (https://github.com/mansoorcheema/unreal_airsim)

## Installation
1. Install [ROS Noetic](http://wiki.ros.org/ROS/Installation)
2.  Install system dependencies: 
```shell script
sudo apt-get install python-wstool python-catkin-tools
```
3. Setup catkin workspace using [catkin-tools](https://catkin-tools.readthedocs.io/en/latest/)
```shell script
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/noetic 
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```
  
5. Move to the catkin workspace: 
```shell script
cd ~/catkin_ws
source devel/setup.bash
cd src
```




6. Checkout the GitHub repository: 
```shell script
git clone git@github.com:ethz-asl/asldoc-2021-ma-mcheem.git
```
7. Install dependencies
```shell script
sudo apt-get install ros-noetic-cmake-modules ros-noetic-control-toolbox ros-noetic-joy ros-noetic-octomap-ros ros-noetic-geographic-msgs autoconf libyaml-cpp-dev protobuf-compiler libgoogle-glog-dev liblapacke-dev libgeographic-dev

wstool init . ./asldoc-2021-ma-mcheem/.rosinstall # Requires Git SSH
wstool update
```
8. Compile
```shell script
catkin build ssc_3d_planning
```

# Experiments 
### Planning Strategies
This framework provides various planning strategies to incorporate scene completions into the planning pipeline. The configurations are provided in `ssc_planning/cfg/planners` directory. To use the configurations set the planner_configuration_file variable as shown:

#### Conventional
Conventional approach that does not use scene completion for planning. 
```shell script
planner_config=$(find ssc_planning)/ssc_planning/cfg/planners/exploration_planner.yaml
```
#### Conservative
Conservatively use scene completions for non safety critical information planning.
```shell script
planner_config=$(find ssc_planning)/ssc_planning/cfg/planners/exploration_planner_ssc_gain_conservative.yaml
```

#### Cautiously Optimistic
Exploit high confidence scene completions for complete planning pipeline. 
```shell script
planner_config=$(find ssc_planning)/ssc_planning/cfg/planners/exploration_planner_ssc_gain_criteria.yaml
```

### Launch Experiment
1. Start Unreal Engine 
```shell script
cd Unreal_INSTALL_DIR # Move to the unreal install directory
./Engine/Binaries/Linux/UE4Editor your-project-file.uproject -opengl4
```
2. Start SSC Network 
3. Launch planning pipeline with the desired planning configuration specified as argument
 ```
 roslaunch ssc_planning launch_ssc_pipeline.launch planner_config_file:=$planner_config
```

## Evaluations

### Enable Evaluations
To collect evaluation data, [launch the previous example](#Launch-Experiment) with additional argument `eval_directory` specifying the path to log the maps for evaluation
```
 roslaunch ssc_planning launch_ssc_pipeline.launch planner_config_file:=$planner_config eval_directory:=eval_data_dir
```
### Metric Calculation
The evaluation metrics can be calculated by the measured as well as scene completed map using the following command.
```
rosrun ssc_mapping ssc_map_eval_node ground_truth_map.tsdf  target_map.[tsdf,ssc] metrics_output.csv
```


## Credits
* **voxblox**
Oleynikova, H., Taylor, Z., Fehr, M., Siegwart, R.Y., & Nieto, J.I. (2017), "**Voxblox: Incremental 3D Euclidean Signed Distance Fields for on-board MAV planning**", in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 1366-1373, 2017 [[IEEE](https://ieeexplore.ieee.org/document/8202315/) | [ArXiv](https://arxiv.org/pdf/1611.03631.pdf)]
* **active\_3d\_planning**
   Lukas Schmid, Michael Pantic, Raghav Khanna, Lionel Ott, Roland Siegwart, and Juan Nieto, "**An Efficient Sampling-based Method for Online Informative Path Planning in Unknown Environments**", in *IEEE Robotics and Automation Letters*, vol. 5, no. 2, pp. 1500-1507, April 2020 [[IEEE](https://ieeexplore.ieee.org/abstract/document/8968434) | [ArXiv](https://arxiv.org/abs/1909.09548) | [Video](https://www.youtube.com/watch?v=lEadqJ1_8Do)]
