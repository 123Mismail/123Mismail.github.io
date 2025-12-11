---
id: c4-urdf-robot-description
title: "Chapter 4: URDF and Robot Description"
sidebar_label: "C4: URDF"
sidebar_position: 4
---

# Chapter 4: URDF and Robot Description

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Create** URDF files defining robot links, joints, and hierarchical structures
2. **Specify** visual, collision, and inertial properties for robot components
3. **Visualize** robot models in RViz and understand the robot state publisher workflow

## Overview

In Chapters 2-3, we established ROS 2's communication infrastructure—Topics for continuous data, Actions for goal-oriented tasks, and Services for queries. However, humanoid robots require more than communication protocols: they need precise **geometric and kinematic models** that define their physical structure.

**URDF (Unified Robot Description Format)** is the XML-based standard for describing robot geometry, kinematics, dynamics, and visual appearance in ROS 2. When a humanoid robot plans a walking gait, reaches for an object, or avoids obstacles, it relies on its URDF model to compute joint positions, collision boundaries, and center-of-mass trajectories.

This chapter introduces URDF's hierarchical structure of **Links** (rigid bodies) and **Joints** (connections between links). You will create URDF files, specify physical properties for simulation, and visualize robot models in RViz. This knowledge forms the foundation for the kinematics, dynamics, and simulation covered in subsequent modules.

## Key Concepts

- **URDF (Unified Robot Description Format)**: XML-based file format for describing robot mechanical structure, including links, joints, geometry, and physical properties
- **Link**: Rigid body component of a robot (e.g., torso, upper arm, thigh) with visual, collision, and inertial properties
- **Joint**: Connection between two links defining their relative motion (revolute, prismatic, continuous, fixed, planar, floating)
- **Visual**: Geometric representation of a link for rendering and visualization (cylinders, boxes, meshes)
- **Collision**: Simplified geometry used for collision detection during motion planning and simulation
- **Inertial**: Physical properties of a link including mass and inertia tensor required for dynamics simulation
- **Robot State Publisher**: ROS 2 node that reads URDF and publishes TF transforms for all robot links
- **RViz**: 3D visualization tool for viewing robot models, sensor data, and planning results

## URDF File Structure

A URDF file is an XML document with a `<robot>` root element containing `<link>` and `<joint>` definitions:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Material definitions (optional) -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <!-- Link definitions -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <!-- Joint definitions -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
</robot>
```

### Joint Types

| Type | Description | Degrees of Freedom | Use Case |
|------|-------------|-------------------|----------|
| **revolute** | Hinge joint with angle limits | 1 (rotation) | Robot arm joints, knee, elbow |
| **continuous** | Hinge joint without limits | 1 (rotation) | Wheels, continuous rotation joints |
| **prismatic** | Sliding joint | 1 (translation) | Linear actuators, telescoping links |
| **fixed** | No relative motion | 0 | Sensor mounts, structural connections |
| **planar** | Motion in a plane | 2 (translation in XY) | Mobile base constraints |
| **floating** | Unconstrained motion | 6 (3 translation + 3 rotation) | Free-floating base (humanoid torso) |

## Simple Two-Link Robot Example

This URDF defines a simple 2-link robot arm with a revolute joint:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Material definitions -->
  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <!-- Base link (fixed to world) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0"
               iyy="0.002" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Shoulder joint (revolute) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

</robot>
```

### Key Elements Explained

- **`<origin xyz="x y z" rpy="roll pitch yaw"/>`**: Position and orientation of link/joint relative to parent frame
- **`<axis xyz="x y z"/>`**: Rotation axis for revolute/continuous joints or translation direction for prismatic joints
- **`<limit>`**: Joint constraints - `lower`/`upper` (radians or meters), `effort` (torque/force in N⋅m or N), `velocity` (rad/s or m/s)
- **`<inertia>`**: 3×3 inertia tensor - `ixx`, `iyy`, `izz` (diagonal), `ixy`, `ixz`, `iyz` (off-diagonal). Units: kg⋅m²

## Visualizing URDF in RViz

### Launch File for Robot Visualization

This Python launch file loads a URDF, starts the robot state publisher, and launches RViz:

```python
# display_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_arm.urdf'
    )

    # Read URDF content
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('my_robot_description'),
            'rviz',
            'view_robot.rviz'
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
```

### Running the Visualization

```bash
# Build the package
colcon build --packages-select my_robot_description

# Source the workspace
source install/setup.bash

# Launch RViz with robot model
ros2 launch my_robot_description display_robot.launch.py
```

The **Robot State Publisher** reads the URDF and publishes TF (transform) messages for each link. The **Joint State Publisher GUI** provides sliders to manually control joint angles. **RViz** subscribes to TF and renders the robot model in 3D.

## URDF Best Practices for Humanoid Robots

1. **Use meters, kilograms, radians**: URDF follows SI units. Joint limits in radians, masses in kg, lengths in m
2. **Provide realistic inertia values**: Incorrect inertia causes simulation instability. Use CAD tools or online calculators
3. **Separate visual and collision geometry**: Visual can be high-detail meshes; collision should be simplified primitives for performance
4. **Define materials for visualization**: Use `<material>` tags with RGBA colors for better RViz rendering
5. **Follow kinematic chain convention**: Base link → torso → limbs. Parent links defined before child links
6. **Test in RViz before simulation**: Verify link positions, joint axes, and limits before moving to Gazebo

## Summary

URDF provides the geometric and kinematic foundation for ROS 2 robot systems. By defining **Links** (rigid bodies with visual, collision, and inertial properties) and **Joints** (connections with motion constraints), URDF enables motion planning, dynamics simulation, and visualization. The **Robot State Publisher** and **RViz** workflow allows real-time visualization of robot configurations.

In Module 2, we will extend URDF models into simulation environments (Gazebo, Isaac Sim) where collision geometry and inertial properties enable physics-based interaction with virtual worlds. Understanding URDF structure is essential for these advanced applications.

## Review Questions

1. **Conceptual**: Why do humanoid robots require separate visual and collision geometries in their URDF definitions?

2. **Applied**: Modify the `simple_arm` URDF to add a forearm link connected by an elbow joint. What joint type should the elbow use?

3. **Structural**: What are the three required child elements of a `<link>` tag for physics simulation, and what does each specify?
