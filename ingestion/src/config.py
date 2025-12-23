from pydantic_settings import BaseSettings
from typing import Optional


class IngestionSettings(BaseSettings):
    # Source configuration
    source_directory: str = "book/"  # Default Docusaurus book directory
    supported_formats: list = ["md", "mdx"]

    # Chunking configuration
    chunk_size: int = 1000  # characters
    chunk_overlap: int = 200  # characters

    # OpenAI Configuration
    openai_api_key: str
    embedding_model: str = "text-embedding-ada-002"

    # Qdrant Configuration
    qdrant_url: str = "http://localhost:6333"  # Default local Qdrant
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "book_embeddings"

    # Database Configuration
    database_url: str  # Neon Postgres connection string


settings = IngestionSettings()