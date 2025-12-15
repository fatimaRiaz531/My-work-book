from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from uuid import UUID
from sqlalchemy.orm import Session
from ..models.document import Document
from ..config.database import get_qdrant
import logging

logger = logging.getLogger(__name__)


class RetrievalService:
    def __init__(self, qdrant_client: QdrantClient = None):
        self.qdrant_client = qdrant_client or get_qdrant()

    def search_similar(
        self,
        query_embedding: List[float],
        collection_name: str,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar documents based on the query embedding.

        Args:
            query_embedding: The embedding vector to search for
            collection_name: Name of the Qdrant collection to search
            top_k: Number of top results to return
            filters: Optional metadata filters to apply

        Returns:
            List of documents with their metadata and similarity scores
        """
        try:
            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )

                if filter_conditions:
                    qdrant_filters = models.Filter(
                        must=filter_conditions
                    )

            # Perform the search
            search_results = self.qdrant_client.search(
                collection_name=collection_name,
                query_vector=("content", query_embedding),
                limit=top_k,
                query_filter=qdrant_filters,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "source_file": result.payload.get("metadata", {}).get("source_file", ""),
                    "section": result.payload.get("metadata", {}).get("section", ""),
                    "chapter": result.payload.get("metadata", {}).get("chapter", ""),
                    "page_number": result.payload.get("metadata", {}).get("page_number"),
                    "score": result.score
                })

            return results

        except Exception as e:
            logger.error(f"Error during similarity search: {str(e)}")
            raise

    def add_embedding(
        self,
        document_id: str,
        embedding: List[float],
        content: str,
        metadata: Dict[str, Any]
    ) -> bool:
        """
        Add an embedding to the Qdrant collection.

        Args:
            document_id: Unique identifier for the document
            embedding: The embedding vector
            content: The original content
            metadata: Additional metadata to store with the embedding

        Returns:
            True if successful, False otherwise
        """
        try:
            self.qdrant_client.upsert(
                collection_name="book_embeddings",
                points=[
                    models.PointStruct(
                        id=document_id,
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
            logger.error(f"Error adding embedding to Qdrant: {str(e)}")
            return False

    def delete_embedding(self, document_id: str) -> bool:
        """
        Delete an embedding from the Qdrant collection.

        Args:
            document_id: ID of the document to delete

        Returns:
            True if successful, False otherwise
        """
        try:
            self.qdrant_client.delete(
                collection_name="book_embeddings",
                points_selector=models.PointIdsList(
                    points=[document_id]
                )
            )
            return True
        except Exception as e:
            logger.error(f"Error deleting embedding from Qdrant: {str(e)}")
            return False