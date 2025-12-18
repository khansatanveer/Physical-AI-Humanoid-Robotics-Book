from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application settings
    app_env: str = "development"
    app_debug: bool = False
    app_host: str = "127.0.0.1"
    app_port: int = 8000

    # API Keys
    cohere_api_key: str
    qdrant_api_key: str
    qdrant_url: str
    neon_database_url: str

    # Application Configuration
    log_level: str = "info"
    max_content_length: int = 10000
    chunk_size: int = 512
    chunk_overlap: int = 128
    top_k: int = 5
    similarity_threshold: float = 0.5

    # Qdrant settings
    qdrant_collection_name: str = "book_content"
    cluster_id: Optional[str] = None

    class Config:
        env_file = ".env"
        case_sensitive = True


# Create a single instance of settings
settings = Settings()