---
sidebar_position: 2
title: 'Physics Simulation with Gazebo'
---

# Physics Simulation with Gazebo

This chapter introduces you to physics simulation in Gazebo, a powerful 3D simulation environment that enables accurate and realistic simulation of robots in complex scenarios. You'll learn about gravity, collisions, joints, and humanoid dynamics that form the foundation of digital twin technology.

## Understanding Physics Simulation

Physics simulation in Gazebo provides a realistic environment where robots can be tested without the need for physical hardware. This allows for:

- Safe testing of robot behaviors
- Rapid prototyping of robot designs
- Testing in hazardous or difficult-to-reach environments
- Reproducible experiments with consistent conditions

### Key Physics Concepts

Gazebo simulates the physical world using a physics engine (typically ODE, Bullet, or DART). The simulation includes:

- **Gravity**: The force that attracts objects toward each other
- **Collisions**: Detection and response when objects make contact
- **Joints**: Connections between rigid bodies that allow specific types of movement
- **Dynamics**: The behavior of objects under the influence of forces

## Setting up Gravity in Gazebo

Gravity is a fundamental aspect of physics simulation that affects all objects in the environment. By default, Gazebo simulates Earth's gravity (9.81 m/s²), but this can be customized.

### Default Gravity Settings

Gazebo's world files define gravity using the `<gravity>` tag within the `<world>` element:

```xml
<world name="default">
  <gravity>0 0 -9.8</gravity>
  <!-- Other world properties -->
</world>
```

### Customizing Gravity

To simulate different environments (like the moon or zero-gravity space), you can modify the gravity vector:

```xml
<!-- Moon gravity (approximately 1/6 of Earth's) -->
<gravity>0 0 -1.62</gravity>

<!-- Zero gravity (space environment) -->
<gravity>0 0 0</gravity>

<!-- Custom gravity in different directions -->
<gravity>-9.8 0 0</gravity> <!-- Gravity in X direction -->
```

## Collision Detection and Response

Collision detection is crucial for realistic physics simulation. Gazebo uses collision models to detect when objects interact.

### Collision Properties

Each object in Gazebo has collision properties that define how it interacts with other objects:

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>1 1 1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.5</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1e+13</kp>
        <kd>1</kd>
        <max_vel>0.01</max_vel>
        <min_depth>0</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

### Collision Detection Methods

Gazebo supports different collision detection algorithms:

- **ODE (Open Dynamics Engine)**: Default physics engine, good for most applications
- **Bullet**: Provides better performance for complex scenarios
- **DART**: Advanced physics engine with more sophisticated contact handling

## Joints and Their Types

Joints connect rigid bodies and define the allowed motion between them. Understanding joints is essential for creating articulated robots.

### Joint Types in Gazebo

1. **Fixed Joint**: No relative motion between connected bodies
2. **Revolute Joint**: Allows rotation around a single axis
3. **Prismatic Joint**: Allows linear motion along a single axis
4. **Continuous Joint**: Revolute joint without joint limits
5. **Planar Joint**: Motion constrained to a plane
6. **Floating Joint**: No constraints (6 degrees of freedom)

### Joint Configuration Example

```xml
<joint name="hinge_joint" type="revolute">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>0 0 1</xyz> <!-- Rotation around Z-axis -->
    <limit>
      <lower>-1.57</lower> <!-- -90 degrees -->
      <upper>1.57</upper>  <!-- 90 degrees -->
      <effort>1000.0</effort>
      <velocity>1.0</velocity>
    </limit>
    <dynamics>
      <damping>0.1</damping>
      <friction>0.0</friction>
    </dynamics>
  </axis>
  <physics>
    <ode>
      <limit>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
      </limit>
      <suspension>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
      </suspension>
    </ode>
  </physics>
</joint>
```

## Humanoid Dynamics Simulation

Humanoid robots present unique challenges in physics simulation due to their complex structure and the need for stable locomotion.

### Key Considerations for Humanoid Dynamics

1. **Center of Mass**: Critical for balance and stability
2. **Inertia Properties**: Affects how the robot responds to forces
3. **Joint Constraints**: Limiting motion to realistic ranges
4. **Control Systems**: Implementing controllers for stable movement

### Inertia Configuration

Proper inertia properties are essential for realistic humanoid simulation:

```xml
<link name="torso">
  <inertial>
    <mass>10.0</mass>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.4" ixy="0.0" ixz="0.0"
             iyy="0.4" iyz="0.0"
             izz="0.4" />
  </inertial>
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
</link>
```

## Practical Exercise: Creating a Simple Pendulum

Let's create a simple pendulum to understand joints and physics:

1. Create a fixed base link
2. Create a pendulum link
3. Connect them with a revolute joint
4. Add visual and collision properties

```xml
<model name="simple_pendulum">
  <!-- Fixed base -->
  <link name="base">
    <pose>0 0 1 0 0 0</pose>
    <visual name="base_visual">
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision name="base_collision">
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Pendulum bob -->
  <link name="pendulum">
    <inertial>
      <mass>1.0</mass>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.1" />
    </inertial>
    <visual name="pendulum_visual">
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision name="pendulum_collision">
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base and pendulum -->
  <joint name="pivot_joint" type="revolute">
    <parent>base</parent>
    <child>pendulum</child>
    <pose>0 0.1 0 0 0 0</pose>
    <axis>
      <xyz>0 1 0</xyz> <!-- Rotate around Y-axis -->
    </axis>
  </joint>
</model>
```

## Hands-on Exercise: Physics Simulation with a Humanoid Robot

### Objective
Create a simple humanoid robot model with basic physics properties and observe its behavior under gravity.

### Steps
1. Create a URDF model of a simplified humanoid with torso, head, arms, and legs
2. Add appropriate inertial properties to each link
3. Connect the links with appropriate joints
4. Load the model into Gazebo
5. Observe the robot's behavior under gravity

### Expected Outcome
The robot should collapse under gravity, demonstrating the importance of control systems in humanoid robotics.

## Summary

Physics simulation in Gazebo provides a realistic environment for testing humanoid robots. Understanding gravity, collisions, joints, and dynamics is crucial for creating effective digital twins. Proper configuration of these elements ensures that simulation results accurately reflect real-world behavior.

In the next chapter, we'll explore how to design environments and interaction scenarios for your simulated robots.