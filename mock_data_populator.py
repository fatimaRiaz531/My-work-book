#!/usr/bin/env python3
"""
Mock data populator to add sample content to Qdrant for testing the chatbot
"""

import uuid
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import os
from dotenv import load_dotenv

# Load environment
load_dotenv('backend/.env')

# Initialize Qdrant client with in-memory storage
qdrant_client = QdrantClient(":memory:")

# Create collection
collection_name = "book_embeddings"
try:
    qdrant_client.get_collection(collection_name)
    print(f"Collection {collection_name} already exists")
    # Delete existing collection to start fresh
    qdrant_client.delete_collection(collection_name)
    print(f"Deleted existing collection {collection_name}")
except:
    print(f"Collection {collection_name} does not exist, will create new one")

qdrant_client.create_collection(
    collection_name=collection_name,
    vectors_config={
        "content": {
            "size": 1536,  # Size for OpenAI embeddings
            "distance": "Cosine"
        }
    }
)
print(f"Created collection {collection_name}")

# Sample content from the Physical AI & Humanoid Robotics book
sample_contents = [
    {
        "content": "Physical AI is an interdisciplinary field combining artificial intelligence with physical systems like robots. It focuses on creating AI that can understand and interact with the physical world effectively.",
        "metadata": {"source_file": "intro.md", "section": "Introduction", "page_number": 1}
    },
    {
        "content": "Humanoid robots are robots with human-like features and capabilities. They are designed to interact with humans in a natural way and perform tasks in human environments.",
        "metadata": {"source_file": "physical-ai-humanoid.md", "section": "Humanoid Robots", "page_number": 5}
    },
    {
        "content": "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior.",
        "metadata": {"source_file": "module-1-ros2.md", "section": "ROS2 Framework", "page_number": 10}
    },
    {
        "content": "Gazebo is a robot simulator that provides realistic physics simulation and rendering. It's widely used in robotics research and development for testing algorithms before deploying on real robots.",
        "metadata": {"source_file": "module-2-gazebo-unity.md", "section": "Simulation Environments", "page_number": 15}
    },
    {
        "content": "Isaac is NVIDIA's robotics simulator and ecosystem. It provides high-fidelity simulation capabilities and tools for developing and testing AI-powered robots.",
        "metadata": {"source_file": "module-3-isaac.md", "section": "Isaac Robotics", "page_number": 20}
    },
    {
        "content": "Vision-Language-Action (VLA) models are AI systems that can understand visual information, process language commands, and generate appropriate actions. These are crucial for robotics applications.",
        "metadata": {"source_file": "module-4-vla.md", "section": "VLA Models", "page_number": 25}
    },
    {
        "content": "Robotics applications span across various domains including manufacturing, healthcare, agriculture, and service industries. The integration of AI enhances their capabilities significantly.",
        "metadata": {"source_file": "implementation-patterns.md", "section": "Robotics Applications", "page_number": 30}
    },
    {
        "content": "AI agents in robotics can perceive their environment, make decisions, and take actions to achieve specific goals. They often use sensors for perception and actuators for action.",
        "metadata": {"source_file": "ai-features.md", "section": "AI Agents", "page_number": 35}
    }
]

# Create mock embeddings (random vectors of size 1536)
import random

# Upload points to Qdrant with mock embeddings
points = []
for item in sample_contents:
    # Create a random embedding vector of size 1536 (OpenAI embedding size)
    embedding = [random.uniform(-0.1, 0.1) for _ in range(1536)]

    # Create a point for Qdrant
    point = PointStruct(
        id=str(uuid.uuid4()),
        vector={"content": embedding},
        payload={
            "content": item["content"],
            "metadata": item["metadata"]
        }
    )
    points.append(point)

# Upload points to Qdrant
qdrant_client.upsert(
    collection_name=collection_name,
    points=points
)

print(f"Successfully added {len(points)} sample documents to Qdrant collection '{collection_name}'")
print("Your chatbot now has sample content to work with!")
print("\nYou can now start the backend and test the chatbot.")

# Save the client to a file for later use
import pickle
with open('qdrant_client_mock.pkl', 'wb') as f:
    pickle.dump(qdrant_client, f)

print("Qdrant client with sample data saved to qdrant_client_mock.pkl")