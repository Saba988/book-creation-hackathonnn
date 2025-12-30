"""
Cohere embedding module
"""
import cohere
import time
from typing import List, Dict, Any, Optional
import logging

from config import Config
from errors import EmbeddingError
from entities import EmbeddingResult


logger = logging.getLogger(__name__)


class CohereEmbedder:
    """
    Class to handle embedding generation using Cohere API.
    """
    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        """
        Initialize the Cohere embedder.

        Args:
            api_key: Cohere API key (if not provided, uses config)
            model: Cohere model name (if not provided, uses config)
        """
        self.api_key = api_key or Config.COHERE_API_KEY
        self.model = model or Config.COHERE_MODEL

        if not self.api_key:
            raise EmbeddingError("Cohere API key is not configured")

        try:
            self.client = cohere.Client(self.api_key)
        except Exception as e:
            raise EmbeddingError(f"Failed to initialize Cohere client: {str(e)}")

    def generate_embeddings(self, texts: List[str], batch_size: int = 96) -> List[EmbeddingResult]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to generate embeddings for
            batch_size: Number of texts to process in each batch (max 96 for Cohere)

        Returns:
            List of EmbeddingResult objects
        """
        if not texts:
            return []

        # Validate batch size (Cohere has limits)
        if batch_size > 96:
            logger.warning("Cohere batch size should not exceed 96, using 96")
            batch_size = 96

        all_results = []
        start_time = time.time()

        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_start_time = time.time()

            try:
                # Generate embeddings
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_document"  # Appropriate for RAG documents
                )

                # Process the results
                for idx, embedding_vector in enumerate(response.embeddings):
                    text_idx = i + idx
                    processing_time = (time.time() - batch_start_time) * 1000  # Convert to milliseconds

                    result = EmbeddingResult(
                        chunk_id=f"text_{text_idx}",  # Will be updated with actual chunk ID
                        embedding_vector=embedding_vector,
                        text_length=len(batch[idx]),
                        processing_time=processing_time
                    )
                    all_results.append(result)

                logger.info(f"Processed batch {i//batch_size + 1} of embeddings ({len(batch)} texts)")

            except Exception as e:
                logger.error(f"Error generating embeddings for batch {i//batch_size + 1}: {str(e)}")
                # Add error placeholders for failed items
                for idx in range(len(batch)):
                    text_idx = i + idx
                    result = EmbeddingResult(
                        chunk_id=f"text_{text_idx}",
                        embedding_vector=[],
                        text_length=len(batch[idx]),
                        processing_time=0
                    )
                    all_results.append(result)
                continue  # Continue with next batch

        total_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        logger.info(f"Generated embeddings for {len(texts)} texts in {total_time:.2f}ms")

        return all_results

    def generate_embedding(self, text: str) -> EmbeddingResult:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            EmbeddingResult object
        """
        results = self.generate_embeddings([text])
        if results:
            return results[0]
        else:
            # Return an error result if generation failed
            return EmbeddingResult(
                chunk_id="unknown",
                embedding_vector=[],
                text_length=len(text),
                processing_time=0
            )


def generate_embeddings_for_chunks(chunks: List[Dict[str, Any]],
                                 api_key: Optional[str] = None,
                                 model: Optional[str] = None,
                                 batch_size: int = 96) -> List[Dict[str, Any]]:
    """
    Generate embeddings for a list of content chunks.

    Args:
        chunks: List of chunk dictionaries with content
        api_key: Cohere API key (if not provided, uses config)
        model: Cohere model name (if not provided, uses config)
        batch_size: Number of chunks to process in each batch

    Returns:
        List of chunk dictionaries with embeddings added
    """
    if not chunks:
        logger.warning("No chunks provided for embedding generation")
        return []

    # Extract texts from chunks
    texts = [chunk.get('content', '') for chunk in chunks if chunk.get('content')]

    if not texts:
        logger.warning("No content found in chunks for embedding generation")
        return chunks

    embedder = CohereEmbedder(api_key, model)
    embedding_results = embedder.generate_embeddings(texts, batch_size)

    # Update chunks with embeddings
    updated_chunks = []
    for i, chunk in enumerate(chunks):
        if i < len(embedding_results):
            result = embedding_results[i]
            # Update the chunk with the embedding
            updated_chunk = chunk.copy()
            updated_chunk['embedding'] = result.embedding_vector

            # Update the chunk_id in the result to match the actual chunk ID
            result.chunk_id = chunk.get('id', f'chunk_{i}')

            updated_chunks.append(updated_chunk)
        else:
            # If there are more chunks than embedding results, add the original chunk
            updated_chunks.append(chunk)

    logger.info(f"Added embeddings to {len([c for c in updated_chunks if c.get('embedding')])} chunks")
    return updated_chunks


def validate_embedding_config() -> bool:
    """
    Validate that the embedding configuration is correct.

    Returns:
        True if configuration is valid, False otherwise
    """
    try:
        Config.validate()
        if not Config.COHERE_API_KEY:
            logger.error("Cohere API key is not configured")
            return False
        return True
    except ValueError as e:
        logger.error(f"Configuration validation failed: {str(e)}")
        return False