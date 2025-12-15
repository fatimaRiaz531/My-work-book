import logging
from typing import Dict, Any
from qdrant_client import QdrantClient
from sqlalchemy import create_engine, text
from ..config.settings import settings

logger = logging.getLogger(__name__)

def validate_qdrant_connection() -> Dict[str, Any]:
    """
    Validate the connection to Qdrant vector database.

    Returns:
        Dict with connection status and details
    """
    try:
        if settings.qdrant_api_key:
            client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10
            )
        else:
            # For local development
            client = QdrantClient(host="localhost", port=6333, timeout=10)

        # Test connection by getting collections
        collections = client.get_collections()

        # Check if our collection exists
        collection_exists = any(
            col.name == settings.qdrant_collection_name
            for col in collections.collections
        )

        return {
            "status": "connected",
            "collections_count": len(collections.collections),
            "target_collection_exists": collection_exists,
            "collection_name": settings.qdrant_collection_name
        }
    except Exception as e:
        logger.error(f"Qdrant connection validation failed: {str(e)}")
        return {
            "status": "error",
            "error": str(e)
        }


def validate_postgres_connection() -> Dict[str, Any]:
    """
    Validate the connection to PostgreSQL/Neon database.

    Returns:
        Dict with connection status and details
    """
    try:
        engine = create_engine(settings.database_url, echo=False)

        # Test connection by executing a simple query
        with engine.connect() as connection:
            result = connection.execute(text("SELECT 1"))
            row = result.fetchone()

        # Verify the connection worked
        if row and row[0] == 1:
            return {
                "status": "connected",
                "test_query": "SELECT 1",
                "result": row[0]
            }
        else:
            return {
                "status": "error",
                "error": "Test query did not return expected result"
            }

    except Exception as e:
        logger.error(f"PostgreSQL/Neon connection validation failed: {str(e)}")
        return {
            "status": "error",
            "error": str(e)
        }


def validate_all_connections() -> Dict[str, Any]:
    """
    Validate all database connections (Qdrant and PostgreSQL/Neon).

    Returns:
        Dict with status of all connections
    """
    qdrant_status = validate_qdrant_connection()
    postgres_status = validate_postgres_connection()

    all_connected = (
        qdrant_status["status"] == "connected" and
        postgres_status["status"] == "connected"
    )

    return {
        "overall_status": "healthy" if all_connected else "unhealthy",
        "qdrant": qdrant_status,
        "postgres": postgres_status,
        "timestamp": __import__('datetime').datetime.utcnow().isoformat()
    }


if __name__ == "__main__":
    # Run validation if script is executed directly
    print("Validating database connections...")
    result = validate_all_connections()
    print(f"Validation result: {result}")