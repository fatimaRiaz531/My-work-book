# Prompt Template: Module 2 - Simulation Environments (Gazebo and Unity)

## Task

Generate a comprehensive, technically rigorous chapter on Gazebo and Unity for advanced AI engineers and roboticists.

## Target Specifications

- **Word count:** 4,000 ± 200 words
- **Citation placeholders:** 16 [CITATION_N] inline markers
- **Code examples:** 5 complete, runnable examples
- **Format:** APA 7th edition academic style
- **Tone:** Technical but accessible; assumes prior AI/software engineering knowledge
- **Readability:** Flesch-Kincaid grade 11-13

## Content Structure

### Section 1: Introduction to Simulation in Robotics (~600 words)

**Key points to cover:**

- Why simulation is critical for robotics development
- The role of physics engines
- Simulation fidelity and the sim-to-real gap
- Types of simulation (kinematic, dynamic, sensor)
- Overview of popular robotics simulators

**Citation requirements:** 3 citations

- Cite a foundational paper on robotics simulation
- Include at least one peer-reviewed paper on the sim-to-real gap
- Reference a major robotics project that heavily relied on simulation

---

### Section 2: Gazebo for Robotics Simulation (~1,000 words)

**Key points to cover:**

- Gazebo architecture: server and clients
- World files (.world) and model files (.sdf)
- Physics engines in Gazebo (ODE, Bullet, DART)
- Sensors and plugins
- Integration with ROS/ROS 2
- Building a simulated environment
- Common pitfalls and performance tuning

**Code examples (2 required):**

1. A simple Gazebo world file with a ground plane and a light source.
2. A Gazebo model file for a simple box with physical properties.

**Citation requirements:** 4 citations

- Official Gazebo documentation
- A paper on a project that used Gazebo for simulation
- A tutorial or guide on Gazebo development

---

### Section 3: Unity for Robotics Simulation (~1,000 words)

**Key points to cover:**

- Unity as a game engine and its application in robotics
- The Unity Robotics Hub
- URDF importer and articulation bodies
- Physics in Unity (Nvidia PhysX)
- Creating realistic visual environments
- C# scripting for robot control
- ROS-Unity integration (ROS TCP Connector)

**Code examples (2 required):**

1. A simple C# script to control a robot joint in Unity.
2. A ROS publisher script in C# to send sensor data from Unity.

**Citation requirements:** 3 citations

- Official Unity Robotics Hub documentation
- A paper on a project that used Unity for robotics simulation
- A tutorial or guide on Unity development for robotics

---

### Section 4: Comparing Gazebo and Unity (~600 words)

**Key points to cover:**

- Physics engine comparison (ODE vs. PhysX)
- Visual rendering quality
- ROS integration and support
- Community and ecosystem
- Use cases: when to choose Gazebo vs. Unity
- The future of robotics simulation

**Citation requirements:** 3 citations

- A comparative study of robotics simulators
- A paper that discusses the pros and cons of different physics engines
- An article or blog post from a robotics company on their choice of simulator

---

### Section 5: Hands-On - Building a Simulated Robot (~800 words)

**Key points to cover:**

- Importing a URDF model into both Gazebo and Unity
- Setting up the environment and lighting
- Writing a simple controller to move the robot
- Collecting sensor data from the simulated robot
- Comparing the behavior of the robot in both simulators

**Code examples (1 required):**

1. A Python script using rclpy to control the robot in both simulators.

**Citation requirements:** 3 citations

- A tutorial on URDF import in Gazebo and Unity
- A paper on sim-to-real transfer
- A blog post on a robotics project that used both simulators

---

## Mandatory Requirements

### Every Claim Must Have a Citation

- Use inline placeholders: `[CITATION_1]`, `[CITATION_2]`, etc.
- Example: "Gazebo uses the ODE physics engine by default [CITATION_5], which provides..."
- Citations should be placed after claims, before punctuation where natural.

### Code Examples Must Be:

- **Syntactically correct** (runnable or valid syntax)
- **Self-contained** (import all dependencies at top)
- **Commented** (explain key lines)
- **Production-ready** (error handling, logging, docstrings)

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

- Gazebo Official Documentation: http://gazebosim.org/
- Unity Robotics Hub Documentation: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Papers on robotics simulation from conferences like ICRA, IROS, and RSS.

---

## Tone & Style Guidelines

- **Avoid:** Marketing language, unsubstantiated claims, vague statements
- **Use:** Active voice, concrete examples, technical precision
- **Include:** Edge cases, common mistakes, debugging tips
- **Assume:** Readers know Python and C#, understand software architecture, but are new to Gazebo and Unity.
