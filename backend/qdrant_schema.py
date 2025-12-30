"""
Qdrant collection schema for storing embeddings with metadata
"""
from qdrant_client.http import models
from typing import Dict, Any


def get_rag_collection_schema(vector_size: int = 1024, distance: str = "Cosine") -> Dict[str, Any]:
    """
    Get the schema configuration for the RAG content collection in Qdrant.

    Args:
        vector_size: Size of the embedding vectors
        distance: Distance metric to use ("Cosine", "Euclid", "Dot")

    Returns:
        Dictionary with collection configuration
    """
    return {
        "vectors": models.VectorParams(
            size=vector_size,
            distance=models.Distance[distance]
        ),
        # Define payload schema for metadata
        "payload_schema": {
            "url": models.PayloadSchemaType.KEYWORD,
            "section": models.PayloadSchemaType.KEYWORD,
            "content": models.PayloadSchemaType.TEXT,
            "created_at": models.PayloadSchemaType.DATETIME,
            "metadata": models.PayloadSchemaType.KEYWORD
        }
    }


def create_rag_collection_config(vector_size: int = 1024, distance: str = "Cosine") -> models.CreateCollection:
    """
    Create a collection configuration object for the RAG content collection.

    Args:
        vector_size: Size of the embedding vectors
        distance: Distance metric to use ("Cosine", "Euclid", "Dot")

    Returns:
        CreateCollection configuration object
    """
    return models.CreateCollection(
        vectors_config=models.VectorParams(
            size=vector_size,
            distance=models.Distance[distance]
        )
    )


def get_point_schema() -> Dict[str, Any]:
    """
    Get the schema for individual points in the collection.

    Returns:
        Dictionary with point schema definition
    """
    return {
        "id": "Unique identifier for the chunk",
        "vector": f"Embedding vector of size N",
        "payload": {
            "url": "Source URL of the content (string)",
            "section": "Section/heading from the source (string)",
            "content": "Cleaned text content of the chunk (string)",
            "metadata": {
                "position": "Position of chunk in document (number)",
                "start_idx": "Start index in original content (number)",
                "end_idx": "End index in original content (number)",
                "length": "Length of the chunk content (number)",
                "chunk_index": "Index of this chunk (number)",
                "total_chunks": "Total number of chunks (number)"
            },
            "created_at": "Timestamp when chunk was created (ISO string)"
        }
    }


# Default configuration for RAG content
DEFAULT_RAG_CONFIG = {
    'vector_size': 1024,  # Default size for Cohere embeddings
    'distance': 'Cosine',  # Cosine distance is common for embeddings
    'collection_name': 'rag_content',
    'hnsw_config': {
        'm': 16,  # Max number of edges per node
        'ef_construct': 100,  # Size of the dynamic list for the nearest neighbors
        'full_scan_threshold': 10000  # Use plain search for small collections
    },
    'optimizer_config': {
        'deleted_threshold': 0.2,  # Threshold for garbage collection
        'vacuum_min_vector_number': 1000  # Minimum number of vectors for vacuum
    }
}


def get_default_rag_collection_config(collection_name: str = None) -> Dict[str, Any]:
    """
    Get the default configuration for a RAG content collection.

    Args:
        collection_name: Name of the collection (uses default if not provided)

    Returns:
        Dictionary with complete collection configuration
    """
    config = DEFAULT_RAG_CONFIG.copy()
    if collection_name:
        config['collection_name'] = collection_name

    return {
        'vector_params': models.VectorParams(
            size=config['vector_size'],
            distance=models.Distance[config['distance']]
        ),
        'hnsw_config': models.HnswConfigDiff(**config['hnsw_config']),
        'optimizer_config': models.OptimizersConfigDiff(**config['optimizer_config']),
        'collection_name': config['collection_name']
    }