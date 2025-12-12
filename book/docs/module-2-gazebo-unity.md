---
sidebar_position: 2
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Introduction

Before deploying code to expensive hardware, engineers test in simulation. Digital twins—virtual replicas of physical systems—enable safe, low-cost experimentation. This module covers physics simulation in Gazebo, sensor simulation, and high-fidelity visualization in Unity for human-robot interaction scenarios.

## Learning Outcomes

After completing this module, you will be able to:

- Build simulated robot environments in Gazebo
- Describe robots using SDF and URDF for physics simulation
- Simulate realistic sensors (LiDAR, depth cameras, IMUs)
- Use Unity for photorealistic rendering and VR interaction
- Create safe testing scenarios for robot behaviors before real-world deployment

---

## The Digital Twin Concept in Robotics

A digital twin is a virtual replica of a physical system that mirrors its structure, behavior, and sensor outputs [CITATION_1]. In robotics, digital twins enable engineers to test algorithms, validate control systems, and debug hardware behavior in simulation before risking expensive equipment or human safety [CITATION_2]. This simulation-first approach has become industry standard at major robotics companies: Tesla's Optimus, Boston Dynamics' Atlas, and Unitree's H1 humanoid robots are extensively tested in simulation before real-world deployment.

The fundamental value proposition of digital twins lies in eliminating barriers to experimentation. Real hardware deployments are slow: setup time, safety constraints, limited testing hours, and the risk of hardware damage. In simulation, experiments run at 10x wall-clock speed, with unlimited trial-and-error cycles [CITATION_3]. A humanoid gait optimization that takes weeks of real-world testing might converge in days of simulated learning. Researchers often train reinforcement learning policies entirely in simulation, then transfer them to real robots via domain randomization techniques.

Multiple simulation platforms serve different purposes. Gazebo (open-source, ROS 2–native) excels at physics accuracy and rapid prototyping. V-REP (now CoppeliaSim) provides visual programming interfaces for non-programmers. NVIDIA Isaac Sim (module 3) specializes in AI-powered perception simulation with synthetic data generation. Unity and Unreal Engine offer photorealistic rendering for human-robot interaction, visualization, and virtual reality training. Choosing the right simulator depends on your priorities: maximum physics fidelity (Gazebo), AI/synthetic data (Isaac Sim), or photorealism (Unity).

The workflow progression is critical: first validate algorithms in gazebo with simplified physics, then transfer to high-fidelity simulators, then deploy to real hardware with careful safety monitoring. Skipping simulation steps creates expensive failures. Every humanoid roboticist has stories of untested algorithms causing hardware damage.

---

## Gazebo: Physics Simulation Engine

Gazebo is the de facto standard open-source robotics simulator, used extensively in ROS 2 development and research [CITATION_4]. Gazebo computes physics using rigid body dynamics solvers (ODE, Bullet, DART engines), predicting how robots move when subjected to forces and torques. This computational physics enables validation of control algorithms before hardware deployment.

Gazebo worlds are defined in SDF (Simulation Description Format) files, XML documents specifying models, their positions, physics parameters, and sensor configurations [CITATION_5]. The following minimal Gazebo world defines a simple robot arm suspended in empty space:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="arm_workspace">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Ground plane -->
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
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Include the robot model (defined elsewhere) -->
    <include>
      <uri>model://robot_arm</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

The `<physics>` element configures the simulation backend [CITATION_6]. The `max_step_size` of 0.001 seconds means the physics engine advances in 1 ms increments; smaller steps increase accuracy but slow simulation. `real_time_factor=1.0` means simulated time progresses at wall-clock speed (1 second of simulation per 1 second of real time). Increasing this factor (e.g., to 2.0) runs simulation faster but reduces control loop frequency.

Gazebo connects to ROS 2 through the gazebo_ros bridge, automatically creating topics for physics state, sensors, and actuation. A robot's joint state automatically publishes to `/joint_states`; motor commands from ROS 2 nodes automatically actuate simulated motors. This seamless integration allows identical ROS 2 control code to work in simulation or on real hardware.

Physics simulation accuracy depends critically on model parameters: inertia tensors, friction coefficients, damping values. Incorrect parameters cause simulated robots to behave unrealistically (unstable, too slow, or unresponsive). Tuning simulation parameters to match real hardware is an art: systematically vary friction and damping until the simulated robot's response matches video of the real robot.

---

## URDF vs. SDF for Simulation

Both URDF and SDF describe robot structure, but SDF is the superior format for physics simulation [CITATION_7]. URDF was originally designed for kinematic descriptions (joint configurations, not dynamics). SDF extends URDF's capabilities with richer physics properties: friction models, contact parameters, soft body deformations, and loop closures in kinematic chains.

A humanoid leg described in SDF includes simulation-specific parameters:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="humanoid_leg">
    <!-- Link: thigh -->
    <link name="thigh">
      <inertial>
        <pose>0 0 -0.2 0 0 0</pose>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.02</iyy>
          <iyz>0.0</iyz>
          <izz>0.008</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.8 0.8 0.8 1.0</ambient>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
        <!-- Friction and contact parameters -->
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>       <!-- Friction coefficient -->
              <mu2>0.8</mu2>     <!-- Directional friction -->
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>100000</kp>    <!-- Contact stiffness -->
              <kd>1000</kd>      <!-- Contact damping -->
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <!-- Joint: hip-knee connection -->
    <joint name="knee_pitch" type="revolute">
      <parent>thigh</parent>
      <child>shank</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
      <limit>
        <lower>0.0</lower>
        <upper>2.5</upper>
        <effort>40.0</effort>
        <velocity>3.0</velocity>
      </limit>
      <!-- Friction and damping in joints -->
      <dynamics>
        <damping>5.0</damping>  <!-- Joint viscous damping -->
        <friction>1.0</friction>  <!-- Static friction -->
      </dynamics>
    </joint>

    <link name="shank">
      <!-- Similar to thigh definition -->
    </link>
  </model>
</sdf>
```

The critical simulation parameters are friction coefficients (`mu`), contact stiffness (`kp`), and damping (`damping`). For biped robots, foot friction is crucial: too little friction causes slipping; excessive friction causes unrealistic adherence to ground. Joint damping simulates motor friction and mechanical losses [CITATION_8].

URDF files are typically converted to SDF format for simulation. ROS 2 tools provide automatic conversion, but SDF remains the more accurate format for Gazebo physics simulation.

---

## Sensor Simulation: LiDAR, Depth, IMU

Realistic sensor simulation accelerates algorithm development. A humanoid robot typically has 20+ sensors: depth cameras, LiDAR, IMU, joint encoders, force/torque sensors. Simulating these sensors accurately enables testing perception pipelines in controlled, repeatable scenarios.

Gazebo supports sensor simulation through plugin mechanisms. A simulated LiDAR broadcasts point cloud scans as ROS 2 messages, identical in format to real hardware:

```xml
<!-- LiDAR sensor in Gazebo SDF -->
<sensor name="lidar" type="ray">
  <pose>0 0 0.5 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>0</min_angle>
        <max_angle>6.2831853</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <min_angle>-0.2617994</min_angle>
        <max_angle>0.2617994</max_angle>
      </vertical>
    </scan>
    <!-- Noise modeling -->
    <range>
      <min>0.3</min>
      <max>25.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>  <!-- 2 cm noise -->
    </noise>
  </ray>
  <!-- ROS 2 publication configuration -->
  <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

This simulated LiDAR generates 360 horizontal × 16 vertical samples (5,760 points per scan) with 2 cm Gaussian noise, mimicking real VLP-16 hardware [CITATION_9]. The gazebo_ros plugin automatically publishes scans as ROS 2 messages on `/lidar/scan`.

Depth cameras are similarly simulated. A simulated RealSense depth camera publishes point clouds and depth images:

```xml
<sensor name="depth_camera" type="depth_camera">
  <pose>0.1 0 0.5 0 0 0</pose>
  <camera>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
  <depth_camera>
    <!-- Depth measurement noise -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1 cm depth noise -->
    </noise>
  </depth_camera>
  <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
    </ros>
  </plugin>
</sensor>
```

IMU simulation publishes linear acceleration, angular velocity, and orientation:

```xml
<sensor name="imu" type="imu">
  <pose>0 0 0.2 0 0 0</pose>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>  <!-- Gyroscope noise -->
        </noise>
      </x>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>  <!-- Accelerometer noise -->
        </noise>
      </x>
    </linear_acceleration>
  </imu>
  <plugin name="gazebo_ros_imu_sensor" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/imu</namespace>
    </ros>
  </plugin>
</sensor>
```

Realistic noise modeling (Gaussian, bias, drift) is essential for algorithm robustness. Perception pipelines trained on clean simulated sensor data often fail on real noisy hardware. Modern practice: add realism to simulation [CITATION_10].

---

## Unity for High-Fidelity Visualization and VR

While Gazebo prioritizes physics accuracy, Unity excels at photorealistic rendering and human-robot interaction scenarios. Unity's graphics engine creates visually compelling environments for demonstrations, user studies, and virtual reality training. Many companies use Unity for human-robot interface development before deploying to real robots.

Connecting a ROS 2 robot in Gazebo to a visualization in Unity requires a ROS 2 bridge. A Unity application subscribes to joint states and camera streams from a Gazebo simulation, rendering the robot in a photorealistic environment:

```python
# Python ROS 2 bridge node (example)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import socket
import json

class ROS2UnityBridge(Node):
    """Forward ROS 2 messages to Unity over WebSocket."""

    def __init__(self):
        super().__init__('ros2_unity_bridge')

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # WebSocket connection to Unity
        self.ws_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ws_socket.connect(('localhost', 9000))

    def joint_state_callback(self, msg):
        """Forward joint states to Unity."""
        data = {
            'type': 'joint_state',
            'joints': msg.name,
            'positions': list(msg.position),
            'timestamp': msg.header.stamp.sec
        }
        self.ws_socket.send(json.dumps(data).encode())

def main(args=None):
    rclpy.init(args=args)
    bridge = ROS2UnityBridge()
    rclpy.spin(bridge)
```

This bridge enables synchronized visualization: Gazebo simulates physics and sensors, Unity renders photorealistic graphics. The approach suits human-robot interaction research, where visual feedback critically affects user perception [CITATION_11].

---

## Hands-On: Simulate a Humanoid Walking

Practical simulation combines Gazebo physics, sensor simulation, ROS 2 interfaces, and control algorithms. A complete example simulates a humanoid robot learning to walk [CITATION_12].

First, create the humanoid model (SDF format, 50-link bipedal structure). Second, set up a Gazebo world with ground and gravity. Third, implement a ROS 2 control node that commands joint positions using feedback from simulated sensors.

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class HumanoidWalkController(Node):
    """Generate walking gaits for simulated humanoid."""

    def __init__(self):
        super().__init__('humanoid_walk_controller')

        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Joint command publisher (position control)
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Gait timing
        self.gait_phase = 0.0
        self.gait_period = 2.0  # 2-second walking period
        self.timer = self.create_timer(0.01, self.gait_step)

        # Current joint state
        self.current_positions = [0.0] * 16  # 16 controllable joints

    def joint_callback(self, msg):
        """Store current joint positions for feedback."""
        self.current_positions = list(msg.position[:16])

    def gait_step(self):
        """Compute and command sinusoidal walking gait."""
        dt = 0.01
        self.gait_phase = (self.gait_phase + dt / self.gait_period) % 1.0

        # Simple sinusoidal gait for humanoid walking
        hip_amplitude = 0.2  # radians
        knee_amplitude = 0.3
        ankle_amplitude = 0.1

        # Phase offset for left/right alternation
        left_phase = self.gait_phase
        right_phase = (self.gait_phase + 0.5) % 1.0

        command = Float64MultiArray()
        command.data = [
            # Left leg
            hip_amplitude * np.sin(2 * np.pi * left_phase),    # left hip flexion
            0.0,  # left hip abduction
            knee_amplitude * np.sin(2 * np.pi * (left_phase + 0.25)),
            ankle_amplitude * np.sin(2 * np.pi * left_phase),

            # Right leg
            hip_amplitude * np.sin(2 * np.pi * right_phase),
            0.0,
            knee_amplitude * np.sin(2 * np.pi * (right_phase + 0.25)),
            ankle_amplitude * np.sin(2 * np.pi * right_phase),

            # Upper body (minimal motion)
            0.0, 0.0,  # Arms
            0.0, 0.0,  # Arms
            0.0, 0.0,  # Torso
            0.0, 0.0   # Head
        ]

        self.joint_cmd_pub.publish(command)

        if int(self.gait_phase * 100) % 100 == 0:
            self.get_logger().info(f'Gait phase: {self.gait_phase:.2f}')

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidWalkController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This controller implements a sinusoidal walking pattern [CITATION_13]. In actual deployment, replace sinusoids with learned policies (from reinforcement learning) or computed trajectories (from motion planning algorithms).

Launch the full simulation stack:

```bash
# Terminal 1: Gazebo simulation
gazebo /path/to/humanoid_world.sdf

# Terminal 2: ROS 2 gait controller
ros2 run humanoid_sim humanoid_walk_controller

# Terminal 3: RViz visualization
ros2 run rviz2 rviz2 -d /path/to/humanoid.rviz
```

The humanoid walks in Gazebo, with joint states published to ROS 2, visualized in RViz. Modify the gait parameters and observe how walking efficiency, stability, and energy consumption change. This tight feedback loop between simulation and control is the foundation of modern robot development.

## Next Steps

With digital twin simulation mastered, proceed to Module 3 (NVIDIA Isaac) to learn AI-powered perception simulation and reinforcement learning for humanoid control.

---

## Simulation Workflow

```
Robot Design (URDF/SDF)
    ↓
Gazebo World Setup
    ↓
Physics Tuning
    ↓
Sensor Simulation
    ↓
ROS 2 Interface
    ↓
Control Algorithm Testing
    ↓
Visualization in RViz/Unity
    ↓
Ready for Jetson Deployment
```

---

## Key Concepts Glossary

- **Gazebo:** Open-source physics simulator for robotics
- **SDF:** Simulation Description Format (superior to URDF for simulation)
- **URDF:** Unified Robot Description Format (kinematic descriptions)
- **Physics Engine:** Software computing rigid body dynamics (ODE, PhysX, Bullet)
- **Sensor Plugin:** Simulates sensor output (LiDAR, camera, IMU)
- **Point Cloud:** 3D data from depth sensors or LiDAR
- **Domain Randomization:** Varying simulation parameters to improve robustness

---

## Next Steps

After mastering Gazebo simulation, proceed to **Module 3: The AI-Robot Brain (NVIDIA Isaac)** where you'll learn photorealistic rendering, synthetic data generation, and hardware-accelerated perception.

---

**Word Count:** [Pending generation]  
**Citations:** [22 citations needed - see `.specify/specs/modules/module-2-gazebo-unity.spec.yml`]  
**Code Examples:** [5 examples pending]  
**Last Updated:** December 7, 2025
