"""
Qdrant Cloud integration module
"""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import logging
import time
import uuid

from config import Config
from errors import StorageError
from entities import ContentChunk


logger = logging.getLogger(__name__)


class QdrantStorage:
    """
    Class to handle storage operations with Qdrant Cloud.
    """
    def __init__(self, url: Optional[str] = None, api_key: Optional[str] = None, collection_name: Optional[str] = None,
                 max_retries: int = 3, retry_delay: float = 1.0):
        """
        Initialize the Qdrant storage client.

        Args:
            url: Qdrant Cloud URL (if not provided, uses config)
            api_key: Qdrant API key (if not provided, uses config)
            collection_name: Name of the collection to use (if not provided, uses config)
            max_retries: Maximum number of retries for failed operations
            retry_delay: Initial delay between retries in seconds (exponential backoff)
        """
        self.url = url or Config.QDRANT_URL
        self.api_key = api_key or Config.QDRANT_API_KEY
        self.collection_name = collection_name or Config.QDRANT_COLLECTION_NAME
        self.max_retries = max_retries
        self.retry_delay = retry_delay

        if not self.url or not self.api_key:
            raise StorageError("Qdrant URL and API key are required")

        try:
            # Initialize Qdrant client
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                prefer_grpc=False,  # Using HTTP for better compatibility
                timeout=30  # 30 second timeout for requests
            )
        except Exception as e:
            raise StorageError(f"Failed to initialize Qdrant client: {str(e)}")

    def _execute_with_retry(self, operation, *args, **kwargs):
        """
        Execute an operation with retry logic for handling connectivity issues.

        Args:
            operation: Function to execute
            *args: Arguments to pass to the operation
            **kwargs: Keyword arguments to pass to the operation

        Returns:
            Result of the operation
        """
        last_exception = None

        for attempt in range(self.max_retries):
            try:
                return operation(*args, **kwargs)
            except Exception as e:
                last_exception = e
                if attempt < self.max_retries - 1:  # Don't sleep on the last attempt
                    delay = self.retry_delay * (2 ** attempt)  # Exponential backoff
                    logger.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay}s...")
                    time.sleep(delay)
                else:
                    logger.error(f"All {self.max_retries} attempts failed: {str(e)}")

        # If all attempts failed, raise the last exception
        raise last_exception

    def create_collection(self, vector_size: int = 1024, distance: str = "Cosine") -> bool:
        """
        Create a collection in Qdrant if it doesn't exist.

        Args:
            vector_size: Size of the embedding vectors
            distance: Distance metric to use ("Cosine", "Euclid", "Dot")

        Returns:
            True if collection was created or already exists
        """
        try:
            # Check if collection already exists with retry
            def get_collections_operation():
                return self.client.get_collections()

            collections = self._execute_with_retry(get_collections_operation)
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create the collection with retry
                def create_collection_operation():
                    return self.client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config=models.VectorParams(
                            size=vector_size,
                            distance=models.Distance.COSINE
                        )
                    )

                self._execute_with_retry(create_collection_operation)
                logger.info(f"Created collection '{self.collection_name}' in Qdrant")
            else:
                logger.info(f"Collection '{self.collection_name}' already exists in Qdrant")

            return True

        except Exception as e:
            logger.error(f"Error creating collection '{self.collection_name}': {str(e)}")
            raise StorageError(f"Failed to create collection: {str(e)}")

    def store_embeddings(self, chunks: List[Dict[str, Any]], batch_size: int = 64) -> int:
        """
        Store content chunks with embeddings in Qdrant.

        Args:
            chunks: List of chunk dictionaries with embeddings
            batch_size: Number of chunks to store in each batch

        Returns:
            Number of chunks successfully stored
        """
        if not chunks:
            logger.warning("No chunks provided for storage")
            return 0

        # Filter chunks that have embeddings
        chunks_with_embeddings = [chunk for chunk in chunks if chunk.get('embedding')]

        if not chunks_with_embeddings:
            logger.warning("No chunks with embeddings found for storage")
            return 0

        stored_count = 0

        # Process in batches to respect API limits
        for i in range(0, len(chunks_with_embeddings), batch_size):
            batch = chunks_with_embeddings[i:i + batch_size]

            try:
                # Prepare points for insertion
                points = []
                for chunk in batch:
                    # Create a unique ID for the point if not already present
                    # Qdrant expects point IDs to be integers or UUIDs, not custom strings
                    chunk_id = chunk.get('id', str(uuid.uuid4()))
                    # Convert to UUID if it's not already in UUID format
                    try:
                        point_id = str(uuid.UUID(chunk_id))
                    except ValueError:
                        # If it's not a valid UUID string, generate a new one
                        point_id = str(uuid.uuid4())

                    # Prepare payload with metadata
                    payload = {
                        'url': chunk.get('url', ''),
                        'section': chunk.get('section', ''),
                        'content': chunk.get('content', ''),
                        'metadata': chunk.get('metadata', {}),
                        'created_at': chunk.get('created_at', '')
                    }

                    # Create point
                    point = models.PointStruct(
                        id=point_id,
                        vector=chunk['embedding'],
                        payload=payload
                    )
                    points.append(point)

                # Upload batch to Qdrant with retry mechanism
                def upsert_operation():
                    return self.client.upsert(
                        collection_name=self.collection_name,
                        points=points
                    )

                self._execute_with_retry(upsert_operation)

                stored_count += len(batch)
                logger.info(f"Stored batch {i//batch_size + 1} ({len(batch)} chunks) in Qdrant")

            except Exception as e:
                logger.error(f"Error storing batch {i//batch_size + 1} after {self.max_retries} attempts: {str(e)}")
                continue  # Continue with next batch

        logger.info(f"Successfully stored {stored_count} chunks in Qdrant collection '{self.collection_name}'")
        return stored_count

    def store_chunk(self, chunk: Dict[str, Any]) -> bool:
        """
        Store a single content chunk with embedding in Qdrant.

        Args:
            chunk: Chunk dictionary with embedding

        Returns:
            True if successfully stored
        """
        if not chunk.get('embedding'):
            logger.error("Chunk does not have an embedding to store")
            return False

        try:
            point_id = chunk.get('id', str(uuid.uuid4()))

            # Prepare payload with metadata
            payload = {
                'url': chunk.get('url', ''),
                'section': chunk.get('section', ''),
                'content': chunk.get('content', ''),
                'metadata': chunk.get('metadata', {}),
                'created_at': chunk.get('created_at', '')
            }

            # Create and upload point
            point = models.PointStruct(
                id=point_id,
                vector=chunk['embedding'],
                payload=payload
            )

            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            logger.info(f"Stored chunk {point_id} in Qdrant")
            return True

        except Exception as e:
            logger.error(f"Error storing chunk: {str(e)}")
            raise StorageError(f"Failed to store chunk: {str(e)}")

    def search(self, query_embedding: List[float], limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search for similar content using embeddings.

        Args:
            query_embedding: Embedding vector to search for similar content
            limit: Maximum number of results to return

        Returns:
            List of matching chunks with similarity scores
        """
        try:
            def search_operation():
                return self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=limit
                )

            results = self._execute_with_retry(search_operation)

            # Format results
            formatted_results = []
            for result in results:
                formatted_result = {
                    'id': result.id,
                    'score': result.score,
                    'payload': result.payload,
                    'url': result.payload.get('url', ''),
                    'section': result.payload.get('section', ''),
                    'content': result.payload.get('content', ''),
                    'metadata': result.payload.get('metadata', {})
                }
                formatted_results.append(formatted_result)

            logger.debug(f"Search returned {len(formatted_results)} results")
            return formatted_results

        except Exception as e:
            logger.error(f"Error performing search: {str(e)}")
            raise StorageError(f"Search failed: {str(e)}")

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection information
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'name': collection_info.config.params.vectors.size,
                'vector_size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance,
                'point_count': collection_info.points_count,
                'config': collection_info.config.dict()
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            raise StorageError(f"Failed to get collection info: {str(e)}")

    def delete_collection(self) -> bool:
        """
        Delete the collection (use with caution).

        Returns:
            True if successfully deleted
        """
        try:
            self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted collection '{self.collection_name}' from Qdrant")
            return True
        except Exception as e:
            logger.error(f"Error deleting collection: {str(e)}")
            raise StorageError(f"Failed to delete collection: {str(e)}")


def store_embeddings_in_qdrant(chunks: List[Dict[str, Any]],
                              url: Optional[str] = None,
                              api_key: Optional[str] = None,
                              collection_name: Optional[str] = None,
                              vector_size: int = 1024) -> int:
    """
    Store embeddings in Qdrant Cloud with the provided configuration.

    Args:
        chunks: List of chunk dictionaries with embeddings
        url: Qdrant Cloud URL (if not provided, uses config)
        api_key: Qdrant API key (if not provided, uses config)
        collection_name: Name of the collection to use (if not provided, uses config)
        vector_size: Size of the embedding vectors

    Returns:
        Number of chunks successfully stored
    """
    storage = QdrantStorage(url, api_key, collection_name)

    # Create collection if it doesn't exist
    storage.create_collection(vector_size=vector_size)

    # Store the embeddings
    stored_count = storage.store_embeddings(chunks)

    return stored_count


def validate_storage_config() -> bool:
    """
    Validate that the storage configuration is correct.

    Returns:
        True if configuration is valid, False otherwise
    """
    try:
        Config.validate()
        if not Config.QDRANT_URL or not Config.QDRANT_API_KEY:
            logger.error("Qdrant URL and API key are not configured")
            return False
        return True
    except ValueError as e:
        logger.error(f"Configuration validation failed: {str(e)}")
        return False