const express = require('express');
const cors = require('cors');
const path = require('path');
const app = express();

// Enable CORS for all routes
app.use(cors());
app.use(express.json());

// Mock data based on the book content
const mockBookContent = [
  {
    id: '1',
    content: 'ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.',
    metadata: { source_file: 'module-1-ros2.md', section: 'Introduction to ROS2', page: 1 }
  },
  {
    id: '2',
    content: 'Gazebo and Unity are simulation environments used in robotics. Gazebo provides realistic physics simulation and 3D rendering capabilities, while Unity offers game engine capabilities for more advanced visualizations.',
    metadata: { source_file: 'module-2-gazebo-unity.md', section: 'Simulation Environments', page: 1 }
  },
  {
    id: '3',
    content: 'Isaac is NVIDIA\'s robotics platform that provides tools for developing, simulating, and deploying AI-powered robots. It includes Isaac Sim for simulation and Isaac ROS for deployment.',
    metadata: { source_file: 'module-3-isaac.md', section: 'Isaac Platform', page: 1 }
  },
  {
    id: '4',
    content: 'Vision Language Action (VLA) models combine visual perception, language understanding, and action planning. These models enable robots to understand complex commands and perform tasks in real-world environments.',
    metadata: { source_file: 'module-4-vla.md', section: 'Vision Language Action Models', page: 1 }
  }
];

// Health check endpoint
app.get('/api/v1/health', (req, res) => {
  res.json({ status: 'healthy', service: 'Mock RAG Chatbot API', mock: true });
});

// Mock chat endpoint
app.post('/api/v1/chat', (req, res) => {
  const { message } = req.body;

  // Simple mock responses based on keywords
  let response = "I'm your Book Assistant. Based on the book content, ";
  let sources = [];

  if (message.toLowerCase().includes('ros') || message.toLowerCase().includes('robot')) {
    response += "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that simplify creating complex robot behavior across various platforms.";
    sources = [mockBookContent[0].metadata];
  } else if (message.toLowerCase().includes('gazebo') || message.toLowerCase().includes('unity') || message.toLowerCase().includes('simulation')) {
    response += "Gazebo and Unity are simulation environments used in robotics. Gazebo provides realistic physics simulation and 3D rendering capabilities, while Unity offers game engine capabilities for advanced visualizations.";
    sources = [mockBookContent[1].metadata];
  } else if (message.toLowerCase().includes('isaac') || message.toLowerCase().includes('nvidia')) {
    response += "Isaac is NVIDIA's robotics platform that provides tools for developing, simulating, and deploying AI-powered robots. It includes Isaac Sim for simulation and Isaac ROS for deployment.";
    sources = [mockBookContent[2].metadata];
  } else if (message.toLowerCase().includes('vision') || message.toLowerCase().includes('language') || message.toLowerCase().includes('action') || message.toLowerCase().includes('vla')) {
    response += "Vision Language Action (VLA) models combine visual perception, language understanding, and action planning. These models enable robots to understand complex commands and perform tasks in real-world environments.";
    sources = [mockBookContent[3].metadata];
  } else {
    response += "the book covers various topics including ROS2, simulation environments like Gazebo and Unity, Isaac platform by NVIDIA, and Vision Language Action models. Please ask specific questions about these topics.";
    sources = mockBookContent.map(item => item.metadata);
  }

  setTimeout(() => {
    res.json({
      response: response,
      sources: sources,
      session_id: 'mock-session-' + Date.now(),
      timestamp: new Date().toISOString()
    });
  }, 500); // Simulate API delay
});

// Mock selection-based chat endpoint
app.post('/api/v1/chat/selection', (req, res) => {
  const { message, selection } = req.body;

  const response = `Based on the selected text: "${selection.substring(0, 50)}...", I can provide more detailed information. The book explains that this topic is important for understanding robotics concepts.`;

  setTimeout(() => {
    res.json({
      response: response,
      sources: [{ source_file: 'selected-text', section: 'User Selection' }],
      session_id: 'mock-session-' + Date.now(),
      timestamp: new Date().toISOString()
    });
  }, 500); // Simulate API delay
});

const PORT = process.env.PORT || 8000;
app.listen(PORT, '0.0.0.0', () => {
  console.log(`Mock RAG Chatbot API server running on port ${PORT}`);
});