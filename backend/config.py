import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """Configuration management for the book embeddings pipeline."""

    # Cohere API Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

    # Book Website Configuration
    BOOK_WEBSITE_URL: str = os.getenv("BOOK_WEBSITE_URL", "")

    # Processing Configuration
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", "1000"))
    OVERLAP_SIZE: int = int(os.getenv("OVERLAP_SIZE", "100"))

    # Validation
    @classmethod
    def validate(cls) -> tuple[bool, str]:
        """Validate that required configuration values are present."""
        if not cls.COHERE_API_KEY:
            return False, "COHERE_API_KEY is required"
        if not cls.QDRANT_URL:
            return False, "QDRANT_URL is required"
        if not cls.BOOK_WEBSITE_URL:
            return False, "BOOK_WEBSITE_URL is required"
        if not cls.CHUNK_SIZE > 0:
            return False, "CHUNK_SIZE must be a positive integer"
        if not cls.OVERLAP_SIZE >= 0:
            return False, "OVERLAP_SIZE must be a non-negative integer"

        return True, "Configuration is valid"

def validate_url(url: str) -> bool:
    """Validate if the provided URL is a proper HTTPS URL."""
    import re
    url_pattern = re.compile(
        r'^https://'  # Must start with https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # Domain
        r'localhost|'  # localhost
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # IP
        r'(?::\d+)?'  # Optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return url_pattern.match(url) is not None


def validate_website_url(url: str) -> tuple[bool, str]:
    """Validate that the target website is accessible before starting processing."""
    import requests
    from urllib.parse import urlparse

    try:
        if not validate_url(url):
            return False, f"Invalid URL format: {url}. Must be a valid HTTPS URL."

        response = requests.head(url, timeout=10)
        if response.status_code >= 400:
            return False, f"Website returned error status: {response.status_code}"

        return True, "Website is accessible"
    except Exception as e:
        return False, f"Unable to access website: {str(e)}"