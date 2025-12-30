"""
Utility functions for string handling and validation
"""
import re
from typing import List, Dict, Any
from urllib.parse import urlparse


def validate_url(url: str) -> bool:
    """
    Validate if a string is a properly formatted URL.

    Args:
        url: The URL string to validate

    Returns:
        True if the URL is valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def clean_text(text: str) -> str:
    """
    Clean text by removing extra whitespace and normalizing.

    Args:
        text: The text to clean

    Returns:
        Cleaned text string
    """
    if not text:
        return ""

    # Remove extra whitespace and normalize
    text = re.sub(r'\s+', ' ', text)
    text = text.strip()
    return text


def generate_chunk_id(url: str, chunk_index: int) -> str:
    """
    Generate a unique ID for a content chunk.

    Args:
        url: The source URL
        chunk_index: The index of the chunk

    Returns:
        A unique chunk ID
    """
    import hashlib
    url_hash = hashlib.md5(url.encode()).hexdigest()[:8]
    return f"chunk_{url_hash}_{chunk_index:04d}"


def validate_chunk_size(chunk_size: int, min_size: int = 100, max_size: int = 10000) -> bool:
    """
    Validate chunk size is within acceptable bounds.

    Args:
        chunk_size: The chunk size to validate
        min_size: Minimum allowed chunk size
        max_size: Maximum allowed chunk size

    Returns:
        True if chunk size is valid, False otherwise
    """
    return min_size <= chunk_size <= max_size


def sanitize_for_logging(data: Any) -> Any:
    """
    Sanitize data for logging by removing sensitive information.

    Args:
        data: The data to sanitize

    Returns:
        Sanitized data safe for logging
    """
    if isinstance(data, str):
        # Remove API keys from strings
        data = re.sub(r'API_KEY=\w+', 'API_KEY=[REDACTED]', data)
        data = re.sub(r'key=\w+', 'key=[REDACTED]', data)
        return data
    elif isinstance(data, dict):
        sanitized = {}
        for key, value in data.items():
            if 'key' in key.lower() or 'token' in key.lower() or 'secret' in key.lower():
                sanitized[key] = '[REDACTED]'
            else:
                sanitized[key] = sanitize_for_logging(value)
        return sanitized
    elif isinstance(data, list):
        return [sanitize_for_logging(item) for item in data]
    else:
        return data