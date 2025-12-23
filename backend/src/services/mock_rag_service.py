from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session
from ..services.retrieval_service import RetrievalService
import logging
import re

logger = logging.getLogger(__name__)

class MockRAGService:
    def __init__(self, db: Session, retrieval_service=None):
        self.db = db
        # Don't require a retrieval service since we're mocking everything
        self.retrieval_service = retrieval_service
        # Define comprehensive responses based on the actual book content
        self.knowledge_base = {
            "physical ai": """Physical AI refers to the integration of artificial intelligence with physical systems, particularly robots. It combines perception, reasoning, and action in real-world environments. Physical AI systems must handle uncertainty, adapt to changing conditions, and interact safely with humans and environments. The course focuses on embodied intelligence and AI systems in the physical world, enabling students to design, simulate, and deploy AI-controlled humanoid robots that perceive, reason, and act in real or simulated environments.""",
            "humanoid robotics": """Humanoid robotics is a branch of robotics focused on creating robots with human-like form and behavior. These robots typically have two legs, two arms, and a head, and are designed to interact with human environments and potentially assist humans in various tasks. The course enables students to design, simulate, and deploy AI-controlled humanoid robots that perceive, reason, and act in real or simulated environments.""",
            "ros": """ROS (Robot Operating System 2) is the middleware that connects digital controllers to physical robots. It enables communication between different software components (nodes) running on robots, sensors, and computational platforms. ROS 2 represents a fundamental paradigm shift in robotics middleware, moving from ROS 1's centralized architecture to a distributed, decentralized system. ROS 2 adopts a client-library architecture where high-level programming interfaces (rclpy for Python, rclcpp for C++) abstract away the complexities of underlying network communication. The Data Distribution Service (DDS) standard serves as ROS 2's transport layer, enabling real-time, fault-tolerant publish-subscribe communication.""",
            "gazebo": """Gazebo is a 3D simulation environment for robotics. It provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. Gazebo is commonly used to test robot algorithms before deploying them on real robots. The course covers Gazebo physics simulation, sensor simulation (cameras, LiDAR, IMU), and domain randomization for sim-to-real transfer.""",
            "nvidia isaac": """NVIDIA Isaac is a robotics platform that provides a complete solution for developing, simulating, and deploying AI-powered robots. It includes Isaac Sim for simulation, Isaac ROS for perception and navigation, and Isaac Apps for specific robot applications. The course covers Isaac Sim for AI training, Isaac ROS packages for perception, GPU-accelerated computer vision, SLAM algorithms (NVIDIA Isaac SLAM), and Edge AI deployment on Jetson platforms.""",
            "vision language action": """Vision-Language-Action (VLA) models are AI systems that can process visual input, understand natural language commands, and generate appropriate actions. These models enable robots to follow complex instructions and perform tasks in unstructured environments. The course covers VLA systems including Vision-Language-Action models, Natural language processing for robotics, Task planning and execution, Multimodal perception systems, and Human-robot interaction.""",
            "robotics": """Robotics is an interdisciplinary field that integrates computer science, electrical engineering, and mechanical engineering. It involves design, construction, operation, and application of robots, as well as computer systems for their control, sensory feedback, and information processing. The course curriculum covers ROS 2 fundamentals, Gazebo simulation, NVIDIA Isaac tools, and Vision-Language-Action systems.""",
            "ai": """Artificial Intelligence (AI) in robotics enables robots to perceive their environment, make decisions, learn from experience, and adapt to new situations. AI techniques used in robotics include machine learning, computer vision, natural language processing, and planning algorithms. The course covers AI integration with robotic systems, including GPU-accelerated perception and real-time processing.""",
            "machine learning": """Machine learning in robotics enables robots to improve their performance through experience. Common applications include perception (object recognition, scene understanding), control (adaptive control, reinforcement learning), and planning (path planning, task planning). The course includes machine learning components in the NVIDIA Isaac module and Vision-Language-Action systems.""",
            "computer vision": """Computer vision enables robots to interpret and understand visual information from the world. It's crucial for navigation, object recognition, manipulation, and human-robot interaction. Common techniques include feature detection, object recognition, and scene understanding. The course covers computer vision in the context of ROS 2, Gazebo simulation, and NVIDIA Isaac tools.""",
            "navigation": """Robot navigation involves perceiving the environment, building maps, localizing the robot within those maps, and planning paths to reach goals. Common approaches include SLAM (Simultaneous Localization and Mapping), path planning algorithms, and obstacle avoidance. The course covers navigation in Module 2 and includes SLAM algorithms in the NVIDIA Isaac module.""",
            "manipulation": """Robotic manipulation refers to a robot's ability to interact with objects in its environment. This includes grasping, moving, and repositioning objects. It requires precise control, understanding of object properties, and coordination between perception and action. The course covers manipulation in the context of humanoid control and autonomous task execution.""",
            "control": """Robot control involves algorithms that determine how a robot should move to achieve its goals. This includes trajectory planning, feedback control, and adaptive control techniques that allow robots to operate effectively despite uncertainties in their environment. The course covers robot control in the ROS 2 module and humanoid control systems.""",
            "sensors": """Robots use various sensors to perceive their environment. Common sensors include cameras for vision, LIDAR and ultrasonic sensors for distance measurement, IMUs for orientation, force/torque sensors for manipulation, and GPS for outdoor navigation. The course covers sensor integration and hardware interfaces, including Intel RealSense cameras and IMU sensors.""",
            "ethics": """Robot ethics and AI safety are crucial considerations in robotics development. This includes ensuring robots operate safely, making ethical decisions in uncertain situations, respecting privacy, and considering the societal impact of robotics technologies. The course addresses safety and reliability in autonomous systems.""",
            "module 1": """Module 1: Robotic Nervous System (ROS 2) - Weeks 1-4. Learning Objectives: Understand ROS 2 architecture and communication patterns, develop nodes for robot control and sensor integration, implement action servers for complex behaviors, design message passing for distributed systems. Topics: ROS 2 concepts (nodes, topics, services, actions), Launch files and parameter management, TF transforms and coordinate frames, Real-time control with ROS 2, Multi-robot systems and networking.""",
            "module 2": """Module 2: Digital Twin Simulation (Gazebo & Unity) - Weeks 5-8. Learning Objectives: Create realistic robot models and environments, implement physics-based simulation, develop digital twins for robot testing, validate algorithms in simulation before real-world deployment. Topics: URDF/XACRO robot modeling, Gazebo physics simulation, Unity integration for advanced visualization, Sensor simulation (cameras, LiDAR, IMU), Domain randomization for sim-to-real transfer.""",
            "module 3": """Module 3: AI-Robot Brain (NVIDIA Isaac) - Weeks 9-12. Learning Objectives: Integrate NVIDIA Isaac tools for perception and navigation, implement GPU-accelerated algorithms, deploy AI models on Jetson platforms, optimize for real-time performance. Topics: Isaac Sim for AI training, Isaac ROS packages for perception, GPU-accelerated computer vision, SLAM algorithms (NVIDIA Isaac SLAM), Edge AI deployment on Jetson.""",
            "module 4": """Module 4: Vision-Language-Action (LLMs + Robotics) - Weeks 13-16. Learning Objectives: Integrate large language models with robotic systems, implement natural language understanding for robot commands, develop multimodal perception-action systems, create autonomous task planning and execution. Topics: Vision-Language-Action models, Natural language processing for robotics, Task planning and execution, Multimodal perception systems, Human-robot interaction.""",
            "urdf": """URDF (Unified Robot Description Format) is an XML schema specifying a robot's kinematic and dynamic properties. Every humanoid robot simulation or real-world system requires a URDF file describing links (rigid bodies) and joints (connections between links). The URDF specifies link inertial properties (mass, inertia tensor) essential for dynamics simulation. The course covers URDF in the context of ROS 2 and robot modeling.""",
            "nodes": """In ROS 2, nodes are independent processes that perform computation—publishers of sensor data, subscribers that consume commands, or stateful controllers managing robot behavior. Each node spins at its own frequency, executing callbacks when relevant messages arrive or timers expire. A single humanoid robot might contain 50+ specialized nodes: one reading the IMU, another controlling the right arm, a third implementing the vision pipeline. Nodes communicate exclusively through ROS 2's middleware layer, ensuring process isolation and fault tolerance.""",
            "topics": """In ROS 2, topics implement publish-subscribe (pub-sub) messaging for asynchronous, one-to-many communication. Any node can publish temperature readings to a `/sensor/imu` topic; any other node can subscribe and receive those messages. Topics decouple producers from consumers—the IMU publisher doesn't know or care how many nodes listen to its data. This loose coupling enables rapid prototyping: add a new logging node, it automatically receives all published data without modifying existing code.""",
            "services": """In ROS 2, services implement synchronous request-response patterns for transactional communication. When a motion planner needs to compute a trajectory, it sends a service request to the trajectory solver and blocks until receiving a response. Unlike topics (send-and-forget), services guarantee completion and provide a return value. However, services introduce synchronization points that can compromise real-time performance if response times become unpredictable.""",
            "actions": """In ROS 2, actions extend the service model to handle long-running, cancellable tasks with intermediate feedback. A navigation action might be invoked with the goal "move to pose X," providing periodic feedback about progress ("traveled 50% of distance") and allowing cancellation ("stop moving—obstacle detected") before completion. Actions are essential for humanoid locomotion: the motion planner sends a "walk to target" action and receives continuous feedback about balance stability and foot contacts.""",
            "rclpy": """rclpy is the Python client library for ROS 2, providing Pythonic bindings to core middleware concepts. Practical ROS 2 development relies on the rclpy client library, which provides Pythonic bindings to core middleware concepts. A typical rclpy node inherits from Node, creates publishers/subscribers, and uses rclpy.spin() to process callbacks.""",
            "simulation": """Simulation is a critical component of the course, using Gazebo for physics simulation and Unity for advanced visualization. Students learn to create realistic robot models and environments, implement physics-based simulation, develop digital twins for robot testing, and validate algorithms in simulation before real-world deployment. The course includes URDF/XACRO robot modeling, sensor simulation, and domain randomization for sim-to-real transfer.""",
            "jetson orin": """The course covers deployment on NVIDIA Jetson Orin platforms for edge AI processing. Students learn to set up Jetson Orin development environments, optimize AI models for edge deployment, implement real-time inference, and understand compute constraints. The hardware configuration includes NVIDIA Jetson AGX Orin (64GB), Intel RealSense cameras, IMU sensors, and networking for ROS 2 communication.""",
            "capstone": """The capstone project requires a simulated humanoid robot that: receives voice commands, plans actions using an LLM, navigates using SLAM, detects and manipulates objects, and executes tasks autonomously. Technical specifications include Gazebo + Isaac Sim simulation environment, SLAM-based autonomous navigation, computer vision for object detection, robotic arm/object interaction, LLM-based task planning, and sim-to-real capability via Jetson.""",
            "learning outcomes": """Upon completion, students will be able to: 1. Design and implement ROS 2-based robotic systems, 2. Create and validate digital twins using Gazebo and Unity, 3. Integrate NVIDIA Isaac tools for AI-powered robotics, 4. Deploy AI systems on edge hardware platforms, 5. Implement Vision-Language-Action systems for embodied AI, 6. Execute sim-to-real transfer for robotic applications, 7. Design autonomous robotic systems with natural language interfaces.""",
            "course overview": """The Physical AI & Humanoid Robotics course is themed around Embodied Intelligence and AI Systems in the Physical World. Duration: One semester (14-16 weeks). Level: Capstone/Graduate. Prerequisites: Linear Algebra, Calculus, Programming (Python/C++), Basic AI/ML. The course enables students to design, simulate, and deploy AI-controlled humanoid robots that perceive, reason, and act in real or simulated environments.""",
            "course goal": """The course goal is to enable students to design, simulate, and deploy AI-controlled humanoid robots that perceive, reason, and act in real or simulated environments. Students will learn to integrate ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems to create autonomous humanoid robots.""",
            "core platforms": """The course covers 5 core platforms: 1. ROS 2 (Humble Hawksbill / Iron) for middleware and communication, 2. Gazebo + Unity for Digital Twin simulation, 3. NVIDIA Isaac Sim & Isaac ROS for AI training and perception, 4. Jetson Orin for Edge AI deployment, 5. GPT-based Vision-Language-Action systems for high-level reasoning.""",
            "gpt": """The course integrates large language models for high-level reasoning and task planning. Students learn to integrate GPT-based Vision-Language-Action systems for natural language understanding and task planning, focusing on Vision-Language-Action models for embodied intelligence. The course covers natural language processing for robotics and multimodal perception systems.""",
            "digital twin": """Digital twin concepts are covered in Module 2: Digital Twin Simulation (Gazebo & Unity). Students learn to create realistic robot models and environments, implement physics-based simulation, develop digital twins for robot testing, and validate algorithms in simulation before real-world deployment. The course covers Unity integration for advanced visualization and domain randomization for sim-to-real transfer.""",
            "humanoid control": """Humanoid control is covered in Week 11: Humanoid Control & Manipulation. Students learn to implement humanoid robot control, develop manipulation algorithms, integrate with perception systems, and handle complex kinematics. The course includes understanding of humanoid-specific challenges like maintaining balance and coordinating multiple degrees of freedom.""",
            "edge ai": """Edge AI deployment is covered using NVIDIA Jetson Orin platforms. Students learn to set up Jetson Orin development environments, optimize AI models for edge deployment, implement real-time inference, and understand compute constraints. The course covers sim-to-real transfer capabilities via Jetson platforms.""",
            "task planning": """Task planning is covered in the Vision-Language-Action module. Students learn to integrate large language models with robotic systems, implement natural language understanding for robot commands, develop autonomous task planning and execution, and create multimodal perception-action systems for embodied AI.""",
            "human robot interaction": """Human-robot interaction is covered in the Vision-Language-Action module. Students learn about natural language understanding, multimodal perception systems, and creating autonomous task planning and execution systems that can respond to human commands and interact naturally with humans.""",
            "slam": """SLAM (Simultaneous Localization and Mapping) is covered in Module 3 with NVIDIA Isaac tools. Students learn to implement SLAM algorithms, create navigation stacks, plan paths in dynamic environments, and integrate perception with navigation. The course includes NVIDIA Isaac SLAM algorithms optimized for real-time performance.""",
            "perception": """Perception systems are covered throughout the course, particularly in Module 3 with NVIDIA Isaac ROS packages. Students learn to implement computer vision algorithms, integrate perception with ROS 2, perform object detection and recognition, and apply sensor fusion techniques. GPU-accelerated perception is emphasized using Isaac ROS packages.""",
            "architecture": """The course covers three main architectural approaches: 1. Digital Twin Workstation (On-Premise) with RTX 4090 GPUs, 2. Cloud GPU Architecture using AWS G5/P4 instances or NVIDIA Omniverse Cloud, 3. Edge AI Deployment using Jetson Orin platforms. Each architecture is optimized for different aspects of the development lifecycle.""",
            "voice to action": """The Voice-to-Action pipeline includes Whisper integration for speech recognition with ROS 2, real-time audio processing, command parsing and validation, and error handling for misrecognition. Audio processing covers noise reduction and enhancement, microphone array processing, echo cancellation, and directional audio focusing.""",
            "behavior trees": """Behavior trees are covered as part of ROS 2 Action Execution Pipelines. Students learn hierarchical task structures, conditional execution paths, and dynamic reconfiguration for complex behavior orchestration in autonomous robotic systems.""",
            "assessment": """The course uses continuous assessment (40%) with weekly lab assignments and module-specific projects, a midterm project (20%) with individual module integration, and a capstone project (40%) with final integrated system demonstration, technical report, and sim-to-real validation.""",
            "lab architecture": """The course supports three lab architectures: 1. Digital Twin Workstation (On-Premise) with RTX 4090 GPUs and 64GB+ RAM, 2. Cloud GPU Architecture using AWS G5.xlarge or Azure ND A100 v4 series, 3. Edge AI Deployment with NVIDIA Jetson AGX Orin (64GB) and compatible humanoid platforms.""",
            "middleware": """ROS 2 serves as the middleware layer that facilitates inter-process communication across distributed robotic systems. It uses the Data Distribution Service (DDS) standard as its transport layer, enabling real-time, fault-tolerant publish-subscribe communication with quality-of-service (QoS) guarantees, deadline monitoring, and topic liveliness detection.""",
            "qos": """Quality of Service (QoS) policies in ROS 2 govern message delivery semantics. A camera node publishing high-frequency image frames might use "best effort" delivery with a small queue, accepting occasional dropped frames. Critical safety commands employ "reliable" delivery with large buffers, ensuring no message loss. The qos_profile parameter allows fine-grained control of deadline, lifespan, and durability settings.""",
            "kinematics": """Kinematics concepts are covered in the context of URDF robot modeling and humanoid control. Students learn forward kinematics for determining end-effector positions, and the importance of accurate kinematic models for stable robot control and manipulation tasks.""",
            "dynamics": """Dynamics simulation is covered in the context of Gazebo physics simulation. Students learn about link inertial properties, mass, inertia tensors, and how accurate dynamics models are critical for stable physics simulation and sim-to-real transfer.""",
            "sim to real": """Sim-to-real transfer is a key concept covered throughout the course. Students learn domain randomization techniques in simulation, how to validate algorithms in simulation before real-world deployment, and how to use Jetson platforms for real-world deployment of simulated systems.""",
            "real time": """Real-time performance is emphasized throughout the course, particularly in ROS 2 architecture, NVIDIA Isaac tools, and Jetson edge deployment. Students learn deterministic execution, sub-millisecond latency requirements, 99.9% message delivery reliability, and optimization techniques for real-time robotic systems."""
        }

    def generate_response(self, query: str, context_documents: List[Dict[str, Any]], conversation_history: Optional[List[Dict[str, str]]] = None) -> Dict[str, Any]:
        """
        Generate a response using the mock knowledge base.
        """
        try:
            query_lower = query.lower().strip()

            # Filter out inappropriate content or non-book related queries
            inappropriate_keywords = [
                'badword', 'inappropriate', 'spam', 'random', 'nonsense', 'garbage', 'meaningless',
                'unrelated', 'offtopic', 'irrelevant', 'joke', 'fun', 'game', 'play', 'fool',
                'stupid', 'idiot', 'hate', 'harm', 'kill', 'destroy', 'attack', 'violence'
            ]

            # Check if query contains inappropriate content
            if any(keyword in query_lower for keyword in inappropriate_keywords):
                response_text = "I can only help with questions related to the Physical AI & Humanoid Robotics course content. Please ask something related to the book topics."
            elif query_lower in ['hi', 'hello', 'hey', 'hola', 'good morning', 'good afternoon', 'good evening', 'greetings']:
                response_text = "Hello! I'm your Book Assistant. I can help you with questions about Physical AI, Humanoid Robotics, ROS, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems. What would you like to know about the course?"
            elif query_lower in ['help', 'what can you do', 'what do you do', 'how can you help']:
                response_text = "I can help you with questions about the Physical AI & Humanoid Robotics course. Ask me about topics like ROS, Gazebo, NVIDIA Isaac, Vision-Language-Action models, robot navigation, manipulation, and other course content. I'm here to assist with your learning!"
            elif query_lower in ['chat 1', 'chapter 1', 'module 1', 'first chapter', 'first module']:
                response_text = self.knowledge_base.get('module 1', "Module 1: Robotic Nervous System (ROS 2) - Weeks 1-4. Learning Objectives: Understand ROS 2 architecture and communication patterns, develop nodes for robot control and sensor integration, implement action servers for complex behaviors, design message passing for distributed systems. Topics: ROS 2 concepts (nodes, topics, services, actions), Launch files and parameter management, TF transforms and coordinate frames, Real-time control with ROS 2, Multi-robot systems and networking.")
            elif query_lower in ['chat 2', 'chapter 2', 'module 2', 'second chapter', 'second module']:
                response_text = self.knowledge_base.get('module 2', "Module 2: Digital Twin Simulation (Gazebo & Unity) - Weeks 5-8. Learning Objectives: Create realistic robot models and environments, implement physics-based simulation, develop digital twins for robot testing, validate algorithms in simulation before real-world deployment. Topics: URDF/XACRO robot modeling, Gazebo physics simulation, Unity integration for advanced visualization, Sensor simulation (cameras, LiDAR, IMU), Domain randomization for sim-to-real transfer.")
            elif query_lower in ['chat 3', 'chapter 3', 'module 3', 'third chapter', 'third module']:
                response_text = self.knowledge_base.get('module 3', "Module 3: AI-Robot Brain (NVIDIA Isaac) - Weeks 9-12. Learning Objectives: Integrate NVIDIA Isaac tools for perception and navigation, implement GPU-accelerated algorithms, deploy AI models on Jetson platforms, optimize for real-time performance. Topics: Isaac Sim for AI training, Isaac ROS packages for perception, GPU-accelerated computer vision, SLAM algorithms (NVIDIA Isaac SLAM), Edge AI deployment on Jetson.")
            elif query_lower in ['chat 4', 'chapter 4', 'module 4', 'fourth chapter', 'fourth module']:
                response_text = self.knowledge_base.get('module 4', "Module 4: Vision-Language-Action (LLMs + Robotics) - Weeks 13-16. Learning Objectives: Integrate large language models with robotic systems, implement natural language understanding for robot commands, develop multimodal perception-action systems, create autonomous task planning and execution. Topics: Vision-Language-Action models, Natural language processing for robotics, Task planning and execution, Multimodal perception systems, Human-robot interaction.")
            else:
                # Look for matches in our knowledge base
                response_text = ""
                found_match = False

                # Check for exact matches in knowledge base
                for keyword, content in self.knowledge_base.items():
                    if keyword in query_lower:
                        response_text = content
                        found_match = True
                        break

                # If no exact match, try partial matches
                if not found_match:
                    for keyword, content in self.knowledge_base.items():
                        if any(word in query_lower for word in keyword.split()):
                            response_text = content
                            found_match = True
                            break

                # If still no match, check for common course-related terms
                if not found_match:
                    course_related_terms = ['robot', 'ai', 'artificial intelligence', 'machine learning', 'programming', 'simulation', 'control', 'navigation', 'perception', 'sensors', 'actuators', 'kinematics', 'dynamics', 'planning', 'path planning', 'motion planning', 'computer vision', 'nlp', 'natural language', 'humanoid', 'biped', 'locomotion', 'balance', 'manipulation', 'gripper', 'arm', 'leg', 'footstep', 'walking', 'control system', 'feedback', 'pid', 'trajectory', 'trajectory planning', 'slam', 'localization', 'mapping', 'sensor fusion', 'imu', 'lidar', 'camera', 'ros', 'middleware', 'communication', 'networking', 'distributed', 'system', 'architecture', 'design', 'implementation', 'deployment', 'edge', 'cloud', 'simulation', 'gazebo', 'unity', 'isaac', 'nvidia', 'jetson', 'hardware', 'software', 'integration', 'testing', 'validation', 'verification', 'safety', 'reliability', 'performance', 'optimization', 'efficiency', 'real-time', 'latency', 'throughput', 'bandwidth', 'qos', 'quality of service']

                    if any(term in query_lower for term in course_related_terms):
                        # Provide a general response about the course instead of "Not found"
                        response_text = """The Physical AI & Humanoid Robotics course covers advanced topics in robotics and AI. The course is structured around 4 main modules:

1. Module 1: Robotic Nervous System (ROS 2) - Covers ROS 2 architecture, nodes, topics, services, and actions
2. Module 2: Digital Twin Simulation (Gazebo & Unity) - Focuses on simulation environments and digital twins
3. Module 3: AI-Robot Brain (NVIDIA Isaac) - Integrates NVIDIA tools for perception and navigation
4. Module 4: Vision-Language-Action (LLMs + Robotics) - Combines large language models with robotic systems

Please ask about a specific topic like 'ROS nodes', 'Gazebo simulation', 'NVIDIA Isaac', 'SLAM', 'computer vision', or another specific concept from the course for more detailed information."""
                        found_match = True
                    elif query_lower.replace(' ', '').isalpha() and len(query_lower) > 3:  # Likely a word but not in our knowledge base
                        response_text = f"""I'm not sure about that specific topic in the course material. The Physical AI & Humanoid Robotics course covers topics like ROS, Gazebo, NVIDIA Isaac, Vision-Language-Action systems, SLAM, computer vision, and humanoid robotics. Could you ask about one of these specific areas? For example: 'What is ROS?', 'Explain Gazebo simulation', or 'Tell me about NVIDIA Isaac'."""
                        found_match = True

                # If no match found at all, provide a helpful general response
                if not found_match:
                    response_text = """Hello! I'm your Book Assistant for the Physical AI & Humanoid Robotics course. I can help you with questions about:

- ROS 2 (Robot Operating System)
- Gazebo simulation
- NVIDIA Isaac tools
- Vision-Language-Action systems
- Humanoid robotics
- Physical AI concepts
- Robot navigation, perception, and control

Please ask specific questions about these topics. For example:
- 'What is ROS?'
- 'Explain Gazebo simulation'
- 'What are the 4 modules in this course?'
- 'Tell me about NVIDIA Isaac'
- 'How does SLAM work?'
"""

            # Prepare the response with sources
            sources = []
            if context_documents:
                for doc in context_documents:
                    source_info = {
                        "id": doc.get("id"),
                        "content": doc.get("content", "")[:200] + "..." if len(doc.get("content", "")) > 200 else doc.get("content", ""),
                        "source_file": doc.get("source_file", ""),
                        "section": doc.get("section", ""),
                        "page_number": doc.get("page_number")
                    }
                    sources.append(source_info)

            return {
                "response": response_text,
                "sources": sources
            }

        except Exception as e:
            logger.error(f"Error generating mock response: {str(e)}")
            return {
                "response": "I can only help with questions related to the Physical AI & Humanoid Robotics course. Please ask something specific about the course content.",
                "sources": []
            }

    def query_full_book(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Query using the mock knowledge base.
        """
        try:
            # Create a simple context based on the query
            context_documents = [{
                "id": "mock_context",
                "content": f"Information related to: {query}",
                "source_file": "mock_knowledge_base",
                "section": "general",
                "page_number": None,
                "score": 0.8
            }]

            # Generate response using the knowledge base
            return self.generate_response(query, context_documents)
        except Exception as e:
            logger.error(f"Error in mock full book query: {str(e)}")
            # Provide a helpful response instead of "Not found in provided content"
            return {
                "response": "Hello! I'm your Book Assistant for the Physical AI & Humanoid Robotics course. I can help you with questions about ROS, Gazebo, NVIDIA Isaac, Vision-Language-Action systems, and other course topics. Please ask specific questions about these subjects.",
                "sources": []
            }

    def query_selection_only(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Query only the provided selected text for relevant information.
        """
        try:
            # Create a context document from the selected text
            context_documents = [{
                "id": "selected_text",
                "content": selected_text,
                "source_file": "user_selection",
                "section": "user_selected",
                "page_number": None,
                "score": 1.0  # Perfect match score
            }]

            # Generate response using only the selected text as context
            return self.generate_response(query, context_documents)
        except Exception as e:
            logger.error(f"Error in mock selection-only query: {str(e)}")
            return {
                "response": "I can help you with questions about the selected text. The Physical AI & Humanoid Robotics course covers topics like ROS, Gazebo, NVIDIA Isaac, Vision-Language-Action systems, and other robotics concepts. Please ask specific questions about these subjects.",
                "sources": []
            }