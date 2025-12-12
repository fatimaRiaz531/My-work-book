# Prompt Template: Module 1 - The Robotic Nervous System (ROS 2)

## Task

Generate a comprehensive, technically rigorous chapter on ROS 2 fundamentals for advanced AI engineers and roboticists.

## Target Specifications

- **Word count:** 4,000 ± 200 words
- **Citation placeholders:** 16 [CITATION_N] inline markers
- **Code examples:** 5 complete, runnable examples
- **Format:** APA 7th edition academic style
- **Tone:** Technical but accessible; assumes prior AI/software engineering knowledge
- **Readability:** Flesch-Kincaid grade 11-13

## Content Structure

### Section 1: Introduction to ROS 2 (~600 words)

**Key points to cover:**

- Brief history: ROS 1 → ROS 2
- ROS 2 is middleware, not an OS
- Client-library architecture (rclpy, rclcpp)
- DDS (Data Distribution Service) as transport layer
- When to use ROS 2 vs. alternatives
- Real-world humanoid robot systems using ROS 2

**Citation requirements:** 3 citations

- Cite official ROS 2 documentation or the ROS Journal
- Include at least one peer-reviewed paper on ROS 2 architecture
- Reference a major robot platform (Boston Dynamics, Unitree, etc.)

---

### Section 2: Core Concepts - Nodes, Topics, Services, Actions (~800 words)

**Key points to cover:**

- **Nodes:** Processes that perform computation
  - Spinning and callbacks
  - Best practices for node design
- **Topics:** Asynchronous pub-sub messaging
  - QoS policies (reliability, durability)
  - Latency considerations
- **Services:** Synchronous request-response
  - When to use services vs. topics
  - Blocking behavior and deadlocks
- **Actions:** Long-running, cancellable tasks
  - Goal, feedback, result pattern
  - Why actions matter for robot control
- **Parameters:** Runtime configuration
  - Parameter server in ROS 2
  - Dynamic parameter updates

**Code examples (2 required):**

1. Simple publisher node (temperature sensor)
2. Simple subscriber node (motor controller)

**Citation requirements:** 4 citations

- Official ROS 2 API documentation
- Paper on pub-sub patterns in distributed systems
- Example from a robotics project or paper

---

### Section 3: Building ROS 2 Packages with Python (rclpy) (~1,000 words)

**Key points to cover:**

- Package structure (setup.py, setup.cfg, package.xml)
- Creating your first ROS 2 package
- Entry points and console scripts
- Building with colcon
- Publishing your first message
- Subscribing and filtering
- Service servers and clients
- Debugging tools (ros2 node list, ros2 topic echo, etc.)
- Common pitfalls and best practices
- Performance optimization tips

**Code examples (3 required):**

1. Complete package.xml for a humanoid controller
2. Publisher node with proper initialization and error handling
3. Subscriber + service server combined node

**Citation requirements:** 3 citations

- ROS 2 development guide
- Paper on robotics middleware design
- Documentation from a humanoid robot project

---

### Section 4: URDF - Describing Humanoid Robot Structure (~800 words)

**Key points to cover:**

- URDF XML structure and schema
- Links and joints hierarchy
- Inertia matrices and mass properties
- Collision and visual geometries
- Joint types (revolute, prismatic, continuous, fixed)
- Frames of reference and transformations
- Humanoid-specific considerations:
  - Center of mass for bipedal stability
  - Joint limits and safety margins
  - Symmetry in biped design
- Loading URDF into Gazebo and RViz
- Tools for URDF visualization and validation

**Code examples (2 required):**

1. Simple 3-DOF arm URDF (shoulder, elbow, wrist)
2. Partial humanoid leg (hip, knee, ankle) with realistic inertias

**Citation requirements:** 3 citations

- URDF specification or official documentation
- Paper on humanoid robot kinematics
- Reference humanoid robot (ASIMO, Boston Dynamics Atlas, etc.)

---

### Section 5: Hands-On - Building a ROS 2 Robot Controller (~800 words)

**Key points to cover:**

- Connecting to hardware drivers
- Receiving goal commands from planners
- Sending motor control signals
- Real-time constraints and determinism
- Testing on simulated vs. real hardware
- Integration with Jetson Orin
- Example workflow: Plan → Control → Feedback loop
- Monitoring and diagnostics

**Code examples (3 required):**

1. Mock robot driver (no hardware needed, perfect for testing)
2. Joint position controller with error handling
3. Velocity profiler for smooth acceleration/deceleration

**Citation requirements:** 3 citations

- ROS 2 control framework documentation
- Paper on real-time robotics control
- Hardware documentation (Jetson, motor driver, etc.)

---

## Mandatory Requirements

### Every Claim Must Have a Citation

- Use inline placeholders: `[CITATION_1]`, `[CITATION_2]`, etc.
- Example: "ROS 2 uses DDS for communication [CITATION_5], which provides..."
- Citations should be placed after claims, before punctuation where natural.

### Code Examples Must Be:

- **Syntactically correct** (runnable or valid syntax)
- **Self-contained** (import all dependencies at top)
- **Commented** (explain key lines)
- **Production-ready** (error handling, logging, docstrings)

### Examples of Good Code:

```python
# Example: ROS 2 Publisher Node
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureSensor(Node):
    """Publishes temperature readings from a simulated sensor."""

    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, '/robot/temperature', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Temperature sensor started")

    def timer_callback(self):
        msg = Float32()
        msg.data = 25.5 + (0.1 * (self.get_clock().now().nanoseconds % 1000))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Success Criteria

- [ ] Total word count: 3,800–4,200 words
- [ ] Exactly 16 [CITATION_N] placeholders
- [ ] All 5 code examples are syntactically valid
- [ ] No plagiarism (0% tolerance)
- [ ] Appropriate for technical audience
- [ ] All links to official documentation included
- [ ] Prerequisites clearly stated at beginning

---

## References to Cite (Must Be Real, Verified Sources)

- ROS 2 Official Documentation: https://docs.ros.org/
- The ROS Journal: https://www.roboticsproceedings.org/
- Boston Dynamics documentation for Atlas humanoid
- Unitree Robotics documentation and papers
- IEEE papers on robotics middleware (search: "ROS2 architecture", "DDS robotics")

---

## Tone & Style Guidelines

- **Avoid:** Marketing language, unsubstantiated claims, vague statements
- **Use:** Active voice, concrete examples, technical precision
- **Include:** Edge cases, common mistakes, debugging tips
- **Assume:** Readers know Python, understand software architecture, but are new to ROS 2
