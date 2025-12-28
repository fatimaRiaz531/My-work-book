from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from qdrant_client import QdrantClient
from typing import Optional
from .settings import settings
import logging

logger = logging.getLogger(__name__)

# Postgres Database setup
SQLALCHEMY_DATABASE_URL = settings.database_url

engine = create_engine(SQLALCHEMY_DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base()


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# Qdrant setup
def get_qdrant_client() -> QdrantClient:
    # Use in-memory Qdrant for development to avoid authentication issues
    logger.info("Using in-memory Qdrant for development")
    client = QdrantClient(":memory:")

    # Create collection if it doesn't exist
    try:
        collections = client.get_collections()
        collection_exists = any(col.name == settings.qdrant_collection_name for col in collections.collections)

        if not collection_exists:
            client.create_collection(
                collection_name=settings.qdrant_collection_name,
                vectors_config={
                    "content": {
                        "size": 1536,  # Size for OpenAI embeddings
                        "distance": "Cosine"
                    }
                }
            )
            logger.info(f"Created Qdrant collection: {settings.qdrant_collection_name}")

            # Add sample data for testing if available
            _add_sample_data(client, settings.qdrant_collection_name)
    except Exception as e:
        logger.warning(f"Warning: Could not create Qdrant collection: {e}")
        # Create collection anyway and add sample data
        try:
            client.create_collection(
                collection_name=settings.qdrant_collection_name,
                vectors_config={
                    "content": {
                        "size": 1536,  # Size for OpenAI embeddings
                        "distance": "Cosine"
                    }
                }
            )
            _add_sample_data(client, settings.qdrant_collection_name)
        except Exception as e2:
            logger.error(f"Error creating collection with sample data: {e2}")

    return client


def _add_sample_data(client, collection_name):
    """Add sample data for testing purposes"""
    import os
    import pickle
    import uuid
    from qdrant_client.http.models import PointStruct

    # Check if we have pre-generated sample data
    sample_data_file = "../../../qdrant_client_mock.pkl"  # Corrected path
    if os.path.exists(sample_data_file):
        try:
            # Load the sample data
            with open(sample_data_file, 'rb') as f:
                sample_client = pickle.load(f)

            # Get the sample data from the loaded client
            try:
                # Try to get points from the sample client
                # Get all points from the collection
                sample_points = sample_client.scroll(
                    collection_name="book_embeddings",
                    limit=100  # Should be enough for our sample
                )[0]  # Get the points list

                # Add the sample data to current client
                if sample_points:
                    points = []
                    for point in sample_points:
                        points.append(PointStruct(
                            id=point.id,
                            vector=point.vector,
                            payload=point.payload
                        ))

                    client.upsert(
                        collection_name=collection_name,
                        points=points
                    )
                    logger.info(f"Added {len(points)} sample documents to Qdrant for testing")
                    return  # Exit after adding sample data
                else:
                    logger.warning("No sample points found in the sample data file")
            except Exception as e:
                logger.warning(f"Could not load sample data from file: {e}")
        except Exception as e:
            logger.warning(f"Could not load sample data file: {e}")

    # If no sample file exists or loading failed, add some basic sample content
    logger.info("Adding basic sample content to Qdrant for testing")
    # Add basic sample content directly
    import random

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

    points = []
    for item in sample_contents:
        # Create a random embedding vector of size 1536 (OpenAI embedding size)
        embedding = [random.uniform(-0.1, 0.1) for _ in range(1536)]

        points.append(PointStruct(
            id=str(uuid.uuid4()),
            vector={"content": embedding},
            payload={
                "content": item["content"],
                "metadata": item["metadata"]
            }
        ))

    client.upsert(
        collection_name=collection_name,
        points=points
    )
    logger.info(f"Added {len(points)} basic sample documents to Qdrant")


# Global Qdrant client instance (lazy initialization to avoid startup errors)
_qdrant_client = None

def get_qdrant():
    global _qdrant_client
    if _qdrant_client is None:
        _qdrant_client = get_qdrant_client()
    return _qdrant_client