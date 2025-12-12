---
sidebar_position: 1
---

# Module 1: The Robotic Nervous System (ROS 2)

## Introduction

ROS 2 (Robot Operating System 2) is the middleware that connects digital controllers to physical robots. It enables communication between different software components (nodes) running on robots, sensors, and computational platforms. This module explains ROS 2's architecture, core concepts, and how to build practical Python packages for humanoid robot control.

## Learning Outcomes

After completing this module, you will be able to:

- Understand ROS 2 middleware architecture and design principles
- Create and manage ROS 2 nodes using Python (rclpy)
- Implement publishers, subscribers, and services
- Write URDF descriptions for humanoid robots
- Deploy ROS 2 control systems to Jetson edge hardware

---

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) represents a fundamental paradigm shift in robotics middleware, moving from ROS 1's centralized architecture to a distributed, decentralized system [CITATION_1]. Unlike traditional operating systems, ROS 2 is not an OS in the conventional sense; rather, it functions as a middleware layer that facilitates inter-process communication across distributed robotic systems [CITATION_2]. This distinction is critical for understanding ROS 2's architecture and its suitability for complex, multi-robot environments.

ROS 2 adopts a client-library architecture where high-level programming interfaces (rclpy for Python, rclcpp for C++) abstract away the complexities of underlying network communication. The Data Distribution Service (DDS) standard serves as ROS 2's transport layer, enabling real-time, fault-tolerant publish-subscribe communication [CITATION_3]. DDS was selected specifically because it provides quality-of-service (QoS) guarantees, deadline monitoring, and topic liveliness detection—features essential for safety-critical robotic systems.

The evolution from ROS 1 to ROS 2 was driven by three primary requirements: real-time performance guarantees, security mechanisms, and multi-platform support. ROS 1 relied on a central ROS Master node, creating a single point of failure. ROS 2 eliminated this bottleneck by implementing peer-to-peer discovery, allowing nodes to directly communicate without central coordination. This architectural change proves particularly valuable in humanoid robotics, where real-time motor control commands and sensor feedback must propagate reliably across dozens of processes running simultaneously.

Major humanoid platforms have standardized on ROS 2 for middleware implementation. The Boston Dynamics Atlas, Unitree H1, and Tesla Bot development teams leverage ROS 2's deterministic execution and modular architecture to manage complex perception pipelines, motion planning stacks, and low-level motor controllers [CITATION_4]. These production deployments validate ROS 2's maturity for real-world robotic applications requiring sub-millisecond latency and 99.9% message delivery reliability.

Contemporary roboticists choose ROS 2 when applications demand network transparency (sensors and controllers distributed across multiple machines), real-time guarantees (deterministic message timing), or cross-platform portability (Linux, Windows, macOS, embedded systems). Alternative middleware systems such as YARP, Orocos, or proprietary systems exist but lack ROS 2's ecosystem maturity, community support, and integration with simulation tools like Gazebo and NVIDIA Isaac Sim.

---

## Core Concepts: Nodes, Topics, Services, and Actions

ROS 2's computational model organizes software into four primary abstractions: nodes, topics, services, and actions [CITATION_5]. Understanding these concepts is essential before writing any ROS 2 code.

**Nodes** are independent processes that perform computation—publishers of sensor data, subscribers that consume commands, or stateful controllers managing robot behavior. Each node spins at its own frequency, executing callbacks when relevant messages arrive or timers expire. A single humanoid robot might contain 50+ specialized nodes: one reading the IMU, another controlling the right arm, a third implementing the vision pipeline. Nodes communicate exclusively through ROS 2's middleware layer, ensuring process isolation and fault tolerance [CITATION_6].

**Topics** implement publish-subscribe (pub-sub) messaging for asynchronous, one-to-many communication. Any node can publish temperature readings to a `/sensor/imu` topic; any other node can subscribe and receive those messages. Topics decouple producers from consumers—the IMU publisher doesn't know or care how many nodes listen to its data. This loose coupling enables rapid prototyping: add a new logging node, it automatically receives all published data without modifying existing code.

ROS 2 topics support configurable Quality of Service (QoS) policies that govern message delivery semantics [CITATION_7]. A camera node publishing high-frequency image frames (30 Hz) might use "best effort" delivery with a small queue, accepting occasional dropped frames. In contrast, critical safety commands—such as emergency stop signals—employ "reliable" delivery with large buffers, ensuring no message loss. The qos_profile parameter allows fine-grained control: deadline (maximum time between successive messages), lifespan (how long a message remains valid), and durability (whether late-joining subscribers receive historical data).

**Services** implement synchronous request-response patterns for transactional communication. When a motion planner needs to compute a trajectory, it sends a service request to the trajectory solver and blocks until receiving a response [CITATION_8]. Unlike topics (send-and-forget), services guarantee completion and provide a return value. However, services introduce synchronization points that can compromise real-time performance if response times become unpredictable.

**Actions** extend the service model to handle long-running, cancellable tasks with intermediate feedback. A navigation action might be invoked with the goal "move to pose X," providing periodic feedback about progress ("traveled 50% of distance") and allowing cancellation ("stop moving—obstacle detected") before completion [CITATION_9]. Actions are essential for humanoid locomotion: the motion planner sends a "walk to target" action and receives continuous feedback about balance stability and foot contacts.

**Parameters** provide runtime configuration without code recompilation. A PID controller might expose gains (Kp, Ki, Kd) as parameters, allowing dynamic tuning while the controller executes. ROS 2's parameter API enables read-write access from any node or command-line tools, facilitating rapid experimentation.

---

## Building ROS 2 Packages with Python (rclpy)

Practical ROS 2 development relies on the rclpy client library, which provides Pythonic bindings to core middleware concepts [CITATION_10]. A ROS 2 package is a directory containing source code, configuration files, and metadata describing the package's dependencies and entry points.

The canonical ROS 2 package structure follows this hierarchy:

```
my_robot_controller/
├── package.xml          # Metadata: name, version, dependencies
├── setup.py             # Package configuration and entry points
├── setup.cfg            # Build tool configuration
├── my_robot_controller/
│   ├── __init__.py      # Python package marker
│   ├── controller.py    # Main controller implementation
│   └── utils.py         # Helper functions
└── test/                # Unit tests
```

The `package.xml` file declares all dependencies, enabling the build system (colcon) to compile packages in the correct order. A robot controller package might declare dependencies on ROS 2 core libraries (rclpy), geometry_msgs (coordinate frames), and sensor_msgs (sensor readings).

Creating a ROS 2 node begins with importing rclpy and defining a class inheriting from `Node`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class RobotController(Node):
    """Control humanoid robot by reading IMU and publishing motor commands."""

    def __init__(self):
        super().__init__('robot_controller')

        # Create subscribers for sensor data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/sensor/imu',
            self.imu_callback,
            qos_profile=rclpy.qos.QoSProfile(depth=10)
        )

        # Create publisher for motor commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create a timer to publish control signals at 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)
        self.latest_imu = None

    def imu_callback(self, msg):
        """Callback invoked when IMU data arrives."""
        self.latest_imu = msg
        self.get_logger().debug(f'Received IMU accel: {msg.linear_acceleration}')

    def control_loop(self):
        """Main control loop: read sensors, compute commands, publish."""
        if self.latest_imu is None:
            return

        # Simple stabilization: push against measured acceleration
        command = Twist()
        command.linear.x = -0.1 * self.latest_imu.linear_acceleration.x
        command.linear.y = -0.1 * self.latest_imu.linear_acceleration.y

        self.cmd_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates essential rclpy patterns [CITATION_11]: Node inheritance, subscription callbacks, timer-driven loops, and publisher pattern. The `rclpy.spin()` function blocks execution, repeatedly calling callbacks as messages arrive. For deterministic timing (critical in motor control), use `create_timer()` instead of event-based subscriptions.

Services allow request-response communication. A trajectory solver service processes goal requests:

```python
from example_interfaces.srv import SetBool
from rclpy.node import Node

class TrajectorySolver(Node):
    def __init__(self):
        super().__init__('trajectory_solver')
        self.service = self.create_service(
            SetBool,
            '/compute_trajectory',
            self.compute_callback
        )

    def compute_callback(self, request, response):
        """Handle trajectory computation request."""
        # Expensive computation here
        response.success = True
        response.message = 'Trajectory computed'
        return response
```

Publishing messages from a node requires only calling the publisher's `publish()` method with the appropriate message type. ROS 2 uses strongly-typed messages (geometry_msgs.msg.Pose, sensor_msgs.msg.Image), enforced at publish time, preventing silent data corruption from type mismatches.

---

## URDF: Describing Humanoid Robot Structure

The Unified Robot Description Format (URDF) is an XML schema specifying a robot's kinematic and dynamic properties [CITATION_12]. Every humanoid robot simulation or real-world system requires a URDF file describing links (rigid bodies) and joints (connections between links) [CITATION_13].

A URDF structure maps directly to a kinematic tree. A simplified humanoid leg URDF demonstrates core concepts:

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg">
    <!-- Define a link (rigid body) -->
    <link name="base_link">
        <inertial>
            <mass value="1.5"/>
            <inertia ixx="0.01" ixy="0" ixz="0"
                     iyy="0.01" iyz="0" izz="0.005"/>
        </inertial>
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.3"/>
            </geometry>
            <material name="torso_color">
                <color rgba="0.8 0.8 0.8 1.0"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.3"/>
            </geometry>
        </collision>
    </link>

    <!-- Define a joint connecting two links -->
    <joint name="hip_pitch" type="revolute">
        <parent link="base_link"/>
        <child link="thigh"/>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
        <limit lower="-2.0" upper="2.0" effort="50" velocity="3.0"/>
    </joint>

    <link name="thigh">
        <inertial>
            <mass value="2.0"/>
            <inertia ixx="0.02" ixy="0" ixz="0"
                     iyy="0.02" iyz="0" izz="0.008"/>
        </inertial>
        <visual>
            <geometry>
                <cylinder radius="0.04" length="0.4"/>
            </geometry>
        </visual>
    </link>

    <!-- Knee joint -->
    <joint name="knee_pitch" type="revolute">
        <parent link="thigh"/>
        <child link="shank"/>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="40" velocity="3.0"/>
    </joint>

    <link name="shank">
        <inertial>
            <mass value="1.8"/>
            <inertia ixx="0.015" ixy="0" ixz="0"
                     iyy="0.015" iyz="0" izz="0.006"/>
        </inertial>
        <visual>
            <geometry>
                <cylinder radius="0.035" length="0.38"/>
            </geometry>
        </visual>
    </link>
</robot>
```

The URDF specifies link inertial properties (mass, inertia tensor) essential for dynamics simulation [CITATION_14]. Inertia tensors define how each link resists rotational acceleration—accurate values are critical for stable physics simulation in Gazebo. The `visual` element describes mesh files or primitives for rendering in RViz. The `collision` element defines contact geometry for physics simulation, often simplified (e.g., capsules for limbs) to reduce computation.

Joints connect links via parent-child relationships. The `type="revolute"` specifies a rotating joint with axis `xyz="0 1 0"` (Y-axis rotation). Joint `limit` tags constrain motion: the hip pitch joint rotates between -2.0 and +2.0 radians, with maximum effort (torque) of 50 N⋅m and velocity of 3.0 rad/s. These limits prevent unrealistic motions and reflect physical robot constraints.

Humanoid-specific URDF design requires careful consideration of biped stability. The center of mass must remain within the support polygon (region between feet contact points) to prevent falling. URDF inertia values must accurately reflect hardware; incorrect inertias cause instability in dynamics simulation, manifesting as simulated robots falling despite physically plausible control.

---

## Hands-On: Building a ROS 2 Robot Controller

Practical robot control integrates ROS 2 communication with hardware drivers and control algorithms. A typical architecture receives goal commands, generates motor commands, and monitors sensor feedback for safety.

The following example implements a mock robot driver (no actual hardware required) with simple proportional control [CITATION_15]:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class MockRobotDriver(Node):
    """Simulates a 3-DOF robot arm, responding to velocity commands."""

    def __init__(self):
        super().__init__('mock_robot_driver')

        # Publisher for joint feedback
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Subscriber for velocity commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Simulation state: three joint angles
        self.joint_angles = [0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0]
        self.target_velocities = [0.0, 0.0, 0.0]

        # Control loop at 100 Hz
        self.create_timer(0.01, self.simulation_step)

    def cmd_callback(self, msg):
        """Parse velocity commands and update target velocities."""
        self.target_velocities[0] = msg.linear.x
        self.target_velocities[1] = msg.linear.y
        self.target_velocities[2] = msg.angular.z

    def simulation_step(self):
        """Update joint positions and publish state."""
        dt = 0.01  # 10 ms time step

        # Simple velocity profile: smoothly accelerate toward target
        acceleration_limit = 0.5  # rad/s^2
        for i in range(3):
            velocity_error = self.target_velocities[i] - self.joint_velocities[i]
            acceleration = max(
                min(velocity_error, acceleration_limit),
                -acceleration_limit
            )
            self.joint_velocities[i] += acceleration * dt
            self.joint_angles[i] += self.joint_velocities[i] * dt

        # Publish joint state
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.joint_angles
        msg.velocity = self.joint_velocities
        msg.effort = [0.0, 0.0, 0.0]

        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    driver = MockRobotDriver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This mock driver accepts velocity commands and simulates joint position changes via numerical integration. Real hardware drivers follow the same pattern: subscribe to command topics, apply control algorithms, and publish sensor feedback. Deploying to Jetson edge hardware requires only changing the hardware interface layer while keeping ROS 2 communication identical.

## Next Steps

After mastering ROS 2 fundamentals, proceed to Module 2 (Digital Twins in Gazebo and Unity) to learn simulation-based testing before deploying to real robots. The concepts introduced here—nodes, topics, packages, and URDF—form the foundation for all subsequent modules.

---

## Key Concepts Glossary

- **Node:** A ROS 2 process performing computation
- **Topic:** Asynchronous publish-subscribe channel for messaging
- **Service:** Synchronous request-response communication pattern
- **Action:** Long-running tasks with goal, feedback, and result
- **URDF:** XML format for describing robot structure (links and joints)
- **DDS:** Data Distribution Service, the underlying transport layer in ROS 2
- **rclpy:** Python client library for ROS 2

---

## Next Steps

After mastering ROS 2 fundamentals, proceed to **Module 2: The Digital Twin (Gazebo & Unity)** where you'll learn to simulate these ROS 2 systems in physics engines before deploying to hardware.

---

**Word Count:** [Pending generation]  
**Citations:** [16 citations needed - see `.specify/specs/modules/module-1-ros2.spec.yml`]  
**Code Examples:** [5 examples pending]  
**Last Updated:** December 7, 2025
