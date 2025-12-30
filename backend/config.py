"""
Configuration module to handle API keys and settings
"""
import os
from dotenv import load_dotenv


# Load environment variables from .env file
load_dotenv()


class Config:
    """Configuration class to handle API keys and settings."""

    # Cohere API Configuration
    COHERE_API_KEY = os.getenv('COHERE_API_KEY')
    COHERE_MODEL = os.getenv('COHERE_MODEL', 'embed-multilingual-v3.0')

    # Qdrant Cloud Configuration
    QDRANT_URL = os.getenv('QDRANT_URL')
    QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
    QDRANT_COLLECTION_NAME = os.getenv('QDRANT_COLLECTION_NAME', 'rag_content')

    # Processing Configuration
    CHUNK_SIZE = int(os.getenv('CHUNK_SIZE', '1000'))
    CHUNK_OVERLAP = int(os.getenv('CHUNK_OVERLAP', '200'))

    # Validation
    @classmethod
    def validate(cls):
        """Validate that all required configuration values are present."""
        errors = []

        if not cls.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is not set")
        if not cls.QDRANT_URL:
            errors.append("QDRANT_URL is not set")
        if not cls.QDRANT_API_KEY:
            errors.append("QDRANT_API_KEY is not set")

        if errors:
            raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")