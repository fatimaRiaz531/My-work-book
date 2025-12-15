from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)


class EmbeddingService:
    def __init__(self, qdrant_client: QdrantClient):
        self.qdrant_client = qdrant_client
        self.openai_client = OpenAI(api_key=settings.openai_api_key)

    def create_embedding(self, text: str) -> List[float]:
        """
        Create an embedding for the given text using OpenAI API.

        Args:
            text: The text to create an embedding for

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-ada-002"
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error creating embedding: {str(e)}")
            raise

    def store_embedding(
        self,
        embedding_id: str,
        embedding: List[float],
        content: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Store an embedding in Qdrant.

        Args:
            embedding_id: Unique identifier for the embedding
            embedding: The embedding vector
            content: The original content
            metadata: Additional metadata to store with the embedding

        Returns:
            True if successful, False otherwise
        """
        try:
            if metadata is None:
                metadata = {}

            self.qdrant_client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=[
                    models.PointStruct(
                        id=embedding_id,
                        vector={"content": embedding},
                        payload={
                            "content": content,
                            "metadata": metadata
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            logger.error(f"Error storing embedding: {str(e)}")
            return False

    def batch_store_embeddings(
        self,
        embeddings_data: List[Dict[str, Any]]
    ) -> bool:
        """
        Store multiple embeddings in Qdrant in a batch operation.

        Args:
            embeddings_data: List of dictionaries containing 'id', 'embedding', 'content', and 'metadata'

        Returns:
            True if successful, False otherwise
        """
        try:
            points = []
            for data in embeddings_data:
                points.append(
                    models.PointStruct(
                        id=data["id"],
                        vector={"content": data["embedding"]},
                        payload={
                            "content": data["content"],
                            "metadata": data.get("metadata", {})
                        }
                    )
                )

            self.qdrant_client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=points
            )
            return True
        except Exception as e:
            logger.error(f"Error storing batch embeddings: {str(e)}")
            return False

    def delete_embedding(self, embedding_id: str) -> bool:
        """
        Delete an embedding from Qdrant.

        Args:
            embedding_id: ID of the embedding to delete

        Returns:
            True if successful, False otherwise
        """
        try:
            self.qdrant_client.delete(
                collection_name=settings.qdrant_collection_name,
                points_selector=models.PointIdsList(
                    points=[embedding_id]
                )
            )
            return True
        except Exception as e:
            logger.error(f"Error deleting embedding: {str(e)}")
            return False