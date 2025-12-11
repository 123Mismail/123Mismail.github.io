---
id: c5-gazebo-simulation
title: "Chapter 5: Gazebo Simulation"
sidebar_label: "C5: Gazebo"
sidebar_position: 5
---

# Chapter 5: Gazebo Simulation

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Configure** Gazebo simulation environments with custom world files and physics properties
2. **Spawn** URDF robot models in Gazebo and integrate with ROS 2 control interfaces
3. **Validate** robot behaviors in physics-based simulation before hardware deployment

## Overview

In Module 1, we established the ROS 2 foundation: communication patterns (Topics, Actions, Services) and robot modeling (URDF). However, testing humanoid robots on physical hardware is expensive, time-consuming, and potentially dangerous. A bipedal robot executing an untested walking gait can fall and sustain damage. Manipulation algorithms require collision-free validation before operating near humans.

**Gazebo** is a physics-based 3D robot simulator that bridges the gap between software development and hardware deployment. It simulates gravity, friction, collisions, and actuator dynamics, enabling developers to test control algorithms, sensor fusion, and navigation before deploying to real robots. For humanoid robotics, Gazebo validates balance control, joint torque limits, and multi-contact dynamics in a safe, repeatable environment.

This chapter introduces Gazebo simulation workflows integrated with ROS 2. You will create simulation worlds, spawn URDF models with physics properties, and connect Gazebo to ROS 2 control interfaces. By chapter's end, you'll understand how to use simulation as a development accelerator and safety validator.

## Key Concepts

- **Gazebo**: Open-source 3D robot simulator with physics engines (ODE, Bullet, Simbody) for realistic dynamics and sensor simulation
- **SDF (Simulation Description Format)**: XML-based format for describing simulation worlds, models, sensors, and plugins in Gazebo
- **World File**: SDF file defining the simulation environment including lighting, terrain, obstacles, and environmental properties
- **Physics Engine**: Computational system simulating rigid body dynamics, collisions, and constraints (ODE by default in Gazebo)
- **ros_gz_bridge**: ROS 2 package providing bidirectional communication between Gazebo topics and ROS 2 topics
- **Spawn Entity**: Process of loading a URDF or SDF model into a running Gazebo simulation at a specified pose
- **Inertial Properties**: Mass, center of mass, and inertia tensor required for physics-accurate simulation
- **Collision Geometry**: Simplified shapes used for contact detection and physics calculations

## Gazebo Architecture with ROS 2

Gazebo integrates with ROS 2 through the `ros_gz` package suite:

```
┌─────────────────┐         ┌──────────────────┐
│  ROS 2 Nodes    │◄────────┤  ros_gz_bridge   │
│  (Controllers,  │         │  (Topic/Service  │
│   Planners)     │────────►│   Translation)   │
└─────────────────┘         └──────────────────┘
                                    │
                                    ▼
                            ┌──────────────────┐
                            │  Gazebo Sim      │
                            │  (Physics Engine,│
                            │   Rendering)     │
                            └──────────────────┘
```

The **ros_gz_bridge** translates between Gazebo's internal topics (e.g., `/model/robot/cmd_vel`) and ROS 2 topics (e.g., `/cmd_vel`), enabling seamless integration with existing ROS 2 stacks.

## Creating a Gazebo World File

World files define the simulation environment using SDF format:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_test_world">

    <!-- GUI camera configuration -->
    <gui>
      <camera name="user_camera">
        <pose>5 5 3 0 0.4 -2.35</pose>
      </camera>
    </gui>

    <!-- Directional sunlight -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane with collision -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Physics engine configuration -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

  </world>
</sdf>
```

### Key SDF Elements

- **`<pose>`**: Position (x, y, z) and orientation (roll, pitch, yaw) in world frame
- **`<physics>`**: Simulation timestep and real-time factor (1.0 = real-time, >1.0 = faster than real-time)
- **`<surface><friction>`**: Contact friction coefficients (mu = static friction, mu2 = rolling friction)
- **`<static>`**: Fixed objects that don't participate in dynamics (ground, walls)

## Spawning a Robot in Gazebo

This Python launch file spawns a URDF robot model in a Gazebo world:

```python
# spawn_robot_gazebo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot_description = get_package_share_directory('my_robot_description')

    # Paths to world and URDF files
    world_file = os.path.join(pkg_robot_description, 'worlds', 'humanoid_test_world.sdf')
    urdf_file = os.path.join(pkg_robot_description, 'urdf', 'simple_arm.urdf')

    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch Gazebo with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_file}.items()
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_arm',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for /clock topic (simulation time)
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        bridge_clock
    ])
```

### Launch File Workflow

1. **Load Gazebo World**: `gz_sim.launch.py` starts Gazebo with the specified world file
2. **Publish Robot Description**: `robot_state_publisher` loads URDF onto `/robot_description` topic
3. **Spawn Entity**: `ros_gz_sim create` subscribes to `/robot_description` and spawns the model at specified pose
4. **Bridge Clock**: Synchronize ROS 2 time with Gazebo simulation time via `/clock` topic

## Physics Simulation Considerations

### Inertial Properties for Stable Simulation

Gazebo requires accurate inertial properties in URDF. Incorrect values cause instability:

```xml
<inertial>
  <mass value="0.5"/>  <!-- kg -->
  <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- Center of mass offset -->
  <inertia ixx="0.002" ixy="0.0" ixz="0.0"
           iyy="0.002" iyz="0.0" izz="0.0001"/>  <!-- kg⋅m² -->
</inertial>
```

**Common Issues**:
- **Zero or very small inertia**: Robot links jitter or explode
- **Inertia not matching geometry**: Unrealistic motion
- **Missing `<collision>` geometry**: Links fall through ground

### Friction and Contact Models

Ground contact friction prevents slipping during walking gaits:

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>    <!-- Static friction coefficient -->
      <mu2>1.0</mu2>  <!-- Rolling friction coefficient -->
    </ode>
  </friction>
</surface>
```

For humanoid feet: `mu ≥ 0.8` simulates rubber sole on concrete.

## Summary

Gazebo simulation provides a physics-based testing environment for humanoid robots, enabling validation of control algorithms, sensor fusion, and dynamics before hardware deployment. By creating **World files** (SDF environments), spawning **URDF models** with accurate **inertial properties**, and bridging communication via **ros_gz_bridge**, developers can iterate rapidly in simulation.

In Chapter 6, we will extend simulation capabilities with NVIDIA Isaac Sim, introducing GPU-accelerated physics, photorealistic rendering, and synthetic sensor data generation for training machine learning models.

## Review Questions

1. **Conceptual**: Why is accurate inertial specification (mass, inertia tensor) critical for stable humanoid robot simulation in Gazebo?

2. **Applied**: Modify the `spawn_robot_gazebo.launch.py` to spawn the robot at position (2.0, 1.0, 0.5) with a 45-degree yaw rotation.

3. **Structural**: What is the role of `ros_gz_bridge`, and why is it necessary for ROS 2-Gazebo integration?
