import sys
import os
from typing import List, Dict, Any
from .processors.docusaurus_parser import DocusaurusParser
from .processors.text_chunker import TextChunker
from .processors.embedding_generator import EmbeddingGenerator
from .config import settings
from qdrant_client import QdrantClient
from qdrant_client.http import models
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def setup_qdrant_collection():
    """Set up the Qdrant collection for storing embeddings."""
    if settings.qdrant_api_key:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False
        )
    else:
        # For local development
        client = QdrantClient(host="localhost", port=6333)

    # Check if collection exists
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
        else:
            logger.info(f"Qdrant collection {settings.qdrant_collection_name} already exists")

    except Exception as e:
        logger.error(f"Error setting up Qdrant collection: {e}")
        raise

    return client


def run_ingestion_pipeline():
    """Run the complete ingestion pipeline."""
    logger.info("Starting ingestion pipeline...")

    # Set up Qdrant client
    qdrant_client = setup_qdrant_collection()

    try:
        # Step 1: Parse documents
        logger.info("Parsing documents...")
        parser = DocusaurusParser()
        documents = parser.parse_all_files()
        logger.info(f"Parsed {len(documents)} documents")

        # Step 2: Chunk documents
        logger.info("Chunking documents...")
        chunker = TextChunker()
        chunks = chunker.chunk_documents(documents)
        logger.info(f"Created {len(chunks)} text chunks")

        # Step 3: Generate embeddings
        logger.info("Generating embeddings...")
        embedding_generator = EmbeddingGenerator()
        processed_chunks = embedding_generator.process_chunks(chunks)
        logger.info(f"Processed {len(processed_chunks)} chunks with embeddings")

        # Step 4: Store in Qdrant
        logger.info("Storing embeddings in Qdrant...")
        points = []
        for chunk in processed_chunks:
            points.append(
                models.PointStruct(
                    id=chunk["id"],
                    vector={"content": chunk["embedding"]},
                    payload={
                        "content": chunk["content"],
                        "metadata": chunk["metadata"]
                    }
                )
            )

        # Upload in batches for efficiency
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            qdrant_client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=batch
            )
            logger.info(f"Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")

        logger.info(f"Ingestion pipeline completed successfully! Stored {len(processed_chunks)} chunks in Qdrant.")

    except Exception as e:
        logger.error(f"Error in ingestion pipeline: {str(e)}")
        raise

    finally:
        logger.info("Ingestion pipeline finished.")


if __name__ == "__main__":
    run_ingestion_pipeline()