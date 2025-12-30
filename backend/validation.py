"""
Input validation for URLs and configuration parameters
"""
from typing import List, Dict, Any, Union
import re
from urllib.parse import urlparse
import logging


logger = logging.getLogger(__name__)


def validate_urls(urls: List[str]) -> List[Dict[str, Union[str, bool]]]:
    """
    Validate a list of URLs.

    Args:
        urls: List of URL strings to validate

    Returns:
        List of dictionaries with validation results for each URL
    """
    results = []

    for url in urls:
        result = {
            'url': url,
            'is_valid': False,
            'error': None
        }

        try:
            # Basic format validation
            parsed = urlparse(url)
            if not parsed.scheme or not parsed.netloc:
                result['error'] = "Invalid URL format: missing scheme or domain"
            elif parsed.scheme not in ['http', 'https']:
                result['error'] = f"Invalid scheme: {parsed.scheme}. Only http and https are allowed"
            else:
                # Additional checks
                if len(url) > 2048:  # Standard URL length limit
                    result['error'] = "URL exceeds maximum length of 2048 characters"
                elif re.search(r'[<>"{}|\\^`\[\]]', url):  # Check for invalid characters
                    result['error'] = "URL contains invalid characters"
                else:
                    result['is_valid'] = True

        except Exception as e:
            result['error'] = f"Error parsing URL: {str(e)}"

        results.append(result)

    return results


def validate_chunk_parameters(chunk_size: int, chunk_overlap: int,
                            min_size: int = 100, max_size: int = 10000) -> Dict[str, Union[bool, str]]:
    """
    Validate chunking parameters.

    Args:
        chunk_size: Size of each chunk
        chunk_overlap: Overlap between chunks
        min_size: Minimum allowed chunk size
        max_size: Maximum allowed chunk size

    Returns:
        Dictionary with validation result and error message if invalid
    """
    result = {
        'is_valid': True,
        'error': None
    }

    if not isinstance(chunk_size, int) or chunk_size <= 0:
        result['is_valid'] = False
        result['error'] = f"Chunk size must be a positive integer, got {chunk_size}"
    elif chunk_size < min_size:
        result['is_valid'] = False
        result['error'] = f"Chunk size must be at least {min_size}, got {chunk_size}"
    elif chunk_size > max_size:
        result['is_valid'] = False
        result['error'] = f"Chunk size must be at most {max_size}, got {chunk_size}"
    elif not isinstance(chunk_overlap, int) or chunk_overlap < 0:
        result['is_valid'] = False
        result['error'] = f"Chunk overlap must be a non-negative integer, got {chunk_overlap}"
    elif chunk_overlap >= chunk_size:
        result['is_valid'] = False
        result['error'] = f"Chunk overlap ({chunk_overlap}) must be less than chunk size ({chunk_size})"

    if not result['is_valid']:
        logger.warning(f"Chunk parameter validation failed: {result['error']}")

    return result


def validate_cohere_model(model_name: str) -> Dict[str, Union[bool, str]]:
    """
    Validate Cohere model name.

    Args:
        model_name: Name of the Cohere model

    Returns:
        Dictionary with validation result and error message if invalid
    """
    result = {
        'is_valid': True,
        'error': None
    }

    if not model_name or not isinstance(model_name, str):
        result['is_valid'] = False
        result['error'] = f"Model name must be a non-empty string, got {model_name}"
        return result

    # Common Cohere model patterns
    valid_patterns = [
        r'^embed-.*',  # embed-english-v3.0, embed-multilingual-v3.0, etc.
        r'^command-.*',  # command models
        r'^command-light-.*'  # command-light models
    ]

    if not any(re.match(pattern, model_name.lower()) for pattern in valid_patterns):
        result['is_valid'] = False
        result['error'] = f"Model name '{model_name}' does not match expected Cohere model patterns"

    return result


def validate_qdrant_collection_name(collection_name: str) -> Dict[str, Union[bool, str]]:
    """
    Validate Qdrant collection name.

    Args:
        collection_name: Name of the Qdrant collection

    Returns:
        Dictionary with validation result and error message if invalid
    """
    result = {
        'is_valid': True,
        'error': None
    }

    if not collection_name or not isinstance(collection_name, str):
        result['is_valid'] = False
        result['error'] = f"Collection name must be a non-empty string, got {collection_name}"
        return result

    # Qdrant collection name validation rules
    if len(collection_name) > 63:
        result['is_valid'] = False
        result['error'] = "Collection name must be 63 characters or less"
    elif not re.match(r'^[a-zA-Z][a-zA-Z0-9_-]*$', collection_name):
        result['is_valid'] = False
        result['error'] = "Collection name must start with a letter and contain only letters, numbers, hyphens, and underscores"

    return result


def validate_api_key(api_key: str, key_type: str = "generic") -> Dict[str, Union[bool, str]]:
    """
    Validate API key format.

    Args:
        api_key: The API key to validate
        key_type: Type of API key ('cohere', 'qdrant', 'generic')

    Returns:
        Dictionary with validation result and error message if invalid
    """
    result = {
        'is_valid': True,
        'error': None
    }

    if not api_key or not isinstance(api_key, str):
        result['is_valid'] = False
        result['error'] = f"{key_type.title()} API key must be a non-empty string"
        return result

    # Basic validation - check if it looks like a key (not empty and not placeholder)
    if api_key.strip() in ['your_api_key_here', 'your-cohere-api-key', 'your-qdrant-api-key', '']:
        result['is_valid'] = False
        result['error'] = f"{key_type.title()} API key appears to be a placeholder value"

    # For specific key types, add more specific validation
    if key_type.lower() == 'cohere':
        # Cohere keys are typically longer alphanumeric strings
        if len(api_key) < 10 or len(api_key) > 100:
            result['is_valid'] = False
            result['error'] = "Cohere API key length appears invalid"
    elif key_type.lower() == 'qdrant':
        # Qdrant keys can vary but are typically not very short
        if len(api_key) < 5:
            result['is_valid'] = False
            result['error'] = "Qdrant API key length appears invalid"

    return result


def validate_pipeline_config(urls: List[str],
                           chunk_size: int,
                           chunk_overlap: int,
                           cohere_model: str = None,
                           collection_name: str = None) -> Dict[str, Any]:
    """
    Validate the complete pipeline configuration.

    Args:
        urls: List of URLs to process
        chunk_size: Size of text chunks
        chunk_overlap: Overlap between chunks
        cohere_model: Cohere model name
        collection_name: Qdrant collection name

    Returns:
        Dictionary with overall validation result and details for each validation
    """
    if not isinstance(urls, list) or len(urls) == 0:
        return {
            'is_valid': False,
            'error': 'URLs must be a non-empty list',
            'details': {}
        }

    # Validate all components
    url_results = validate_urls(urls)
    chunk_result = validate_chunk_parameters(chunk_size, chunk_overlap)
    model_result = validate_cohere_model(cohere_model) if cohere_model else {'is_valid': True}
    collection_result = validate_qdrant_collection_name(collection_name) if collection_name else {'is_valid': True}

    # Overall result
    all_valid = (
        all(result['is_valid'] for result in url_results) and
        chunk_result['is_valid'] and
        model_result['is_valid'] and
        collection_result['is_valid']
    )

    result = {
        'is_valid': all_valid,
        'details': {
            'urls': url_results,
            'chunk_parameters': chunk_result,
            'cohere_model': model_result,
            'collection_name': collection_result
        }
    }

    if not all_valid:
        result['error'] = "One or more validation checks failed"
        logger.warning("Pipeline configuration validation failed")

    return result


def sanitize_for_logging(data: Union[str, Dict, List]) -> Union[str, Dict, List]:
    """
    Sanitize data for logging by removing sensitive information.

    Args:
        data: Data to sanitize

    Returns:
        Sanitized data safe for logging
    """
    import re

    if isinstance(data, str):
        # Remove API keys from strings
        sanitized = re.sub(r'[a-zA-Z0-9]{20,}', '[REDACTED_API_KEY]', data)
        sanitized = re.sub(r'key=.*?(\s|$)', 'key=[REDACTED] ', sanitized)
        sanitized = re.sub(r'api_key=.*?(\s|$)', 'api_key=[REDACTED] ', sanitized)
        return sanitized
    elif isinstance(data, dict):
        sanitized = {}
        for key, value in data.items():
            if any(token in key.lower() for token in ['key', 'token', 'secret', 'password', 'auth']):
                sanitized[key] = '[REDACTED]'
            else:
                sanitized[key] = sanitize_for_logging(value)
        return sanitized
    elif isinstance(data, list):
        return [sanitize_for_logging(item) for item in data]
    else:
        return data