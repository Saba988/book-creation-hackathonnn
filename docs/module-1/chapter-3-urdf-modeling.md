---
sidebar_position: 4
---

# Chapter 3: URDF Modeling for Humanoid Robots

## Introduction to URDF

**URDF (Unified Robot Description Format)** is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, sensors, and other components. URDF is essential for robot simulation, visualization, and control.

For humanoid robots, URDF files describe the robot's structure including limbs, joints, and physical properties. This allows ROS tools to understand the robot's kinematics and dynamics, which is crucial for motion planning, control, and simulation.

## URDF Structure

A URDF file consists of several key elements:

### Links
**Links** represent rigid bodies of the robot. Each link has:
- Physical properties (mass, inertia)
- Visual properties (shape, color, mesh)
- Collision properties (collision geometry)

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.2 0.2 0.2" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.2 0.2 0.2" />
    </geometry>
  </collision>
</link>
```

### Joints
**Joints** define the connections between links. Common joint types include:
- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint
- **fixed**: Rigid connection (no movement)

```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link" />
  <child link="wheel_link" />
  <origin xyz="0.1 0 0" rpy="0 0 0" />
  <axis xyz="0 1 0" />
</joint>
```

## URDF for Humanoid Robots

Humanoid robots have a specific structure that typically includes:
- Torso/Body
- Head
- Arms (with shoulders, elbows, wrists)
- Legs (with hips, knees, ankles)
- Hands and feet

### Example Humanoid Robot URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </visual>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso" />
    <child link="left_upper_arm" />
    <origin xyz="0.15 0 0.1" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05" />
      </geometry>
    </visual>
  </link>

  <!-- Additional joints and links for the rest of the robot would continue here -->
</robot>
```

## Common URDF Elements for Humanoid Robots

### Sensors
Humanoid robots often include sensors in their URDF:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

### Transmission Elements
For controlling joints in simulation:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_left_shoulder">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Working with URDF Files

### Validating URDF Files
You can validate a URDF file using the check_urdf tool:

```bash
# Check if the URDF is valid
check_urdf /path/to/robot.urdf

# Display the kinematic tree
urdf_to_graphiz /path/to/robot.urdf
```

### Visualizing URDF Files
You can visualize your robot model in RViz:

```bash
# Launch RViz with the robot model
roslaunch urdf_tutorial display.launch model:=/path/to/robot.urdf
```

Or using ROS 2:
```bash
# Using xacro to process and visualize the robot
ros2 run xacro xacro --inorder robot.urdf.xacro | ros2 run rviz2 rviz2
```

## Best Practices for Humanoid Robot URDF

1. **Consistent Naming**: Use a consistent naming convention for links and joints
2. **Proper Inertial Properties**: Ensure mass and inertia values are realistic
3. **Collision vs Visual**: Use simpler geometry for collision detection than for visual representation
4. **Joint Limits**: Set appropriate limits for all revolute joints
5. **Origin Definitions**: Define origins carefully to ensure proper kinematic chains
6. **Gazebo Integration**: Include appropriate Gazebo plugins for simulation

## Practical Exercise

Create a simplified URDF for a humanoid robot that includes:
1. A torso link
2. A head link connected to the torso
3. Two arm links (upper and lower) on each side
4. Proper joint definitions with appropriate limits
5. Basic visual and collision properties

## Common URDF Issues and Solutions

1. **Floating Point Precision**: Use appropriate precision in numerical values
2. **Mass Properties**: Ensure all links have proper mass and inertia
3. **Kinematic Loops**: URDF doesn't support kinematic loops; use separate mechanisms for closed chains
4. **File Organization**: Consider using xacro macros for complex robots

## Summary

URDF is fundamental to working with humanoid robots in ROS. It provides a complete description of the robot's physical structure, enabling simulation, visualization, and control. Understanding how to create and modify URDF files is essential for preparing humanoid robots for simulation and real-world deployment.

When working with humanoid robots, pay special attention to the kinematic structure, ensuring that joint limits and physical properties accurately reflect the real robot's capabilities.