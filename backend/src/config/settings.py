from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API Configuration
    api_title: str = "RAG Chatbot API"
    api_version: str = "1.0.0"
    debug: bool = False

    # OpenAI Configuration
    openai_api_key: str
    openai_model: str = "gpt-3.5-turbo"

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    qdrant_collection_name: str = "book_embeddings"

    # Database Configuration
    database_url: str  # Neon Postgres connection string

    # Security
    secret_key: str
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    # Application settings
    session_expire_minutes: int = 1440  # 24 hours

    class Config:
        env_file = ".env"


settings = Settings()