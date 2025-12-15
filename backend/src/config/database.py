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
    if settings.qdrant_api_key:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Set to True in production for better performance
        )
    else:
        # For local development without API key
        client = QdrantClient(host="localhost", port=6333)

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
    except Exception as e:
        logger.warning(f"Warning: Could not connect to Qdrant during startup: {e}")
        # Don't raise during startup, just return the client
        # Actual connection will be tested when the client is used

    return client


# Global Qdrant client instance (lazy initialization to avoid startup errors)
_qdrant_client = None

def get_qdrant():
    global _qdrant_client
    if _qdrant_client is None:
        _qdrant_client = get_qdrant_client()
    return _qdrant_client