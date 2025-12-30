"""
Pipeline orchestrator class
"""
from typing import List, Dict, Any, Optional
import logging
import time
from datetime import datetime

from ingestion import ingest_urls
from cleaning import clean_content
from chunking import chunk_content
from embedding import generate_embeddings_for_chunks
from storage import store_embeddings_in_qdrant
from entities import PipelineState
from config import Config


logger = logging.getLogger(__name__)


class PipelineOrchestrator:
    """
    Class to orchestrate the complete pipeline from URL ingestion to vector storage.
    """
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200,
                 cohere_model: str = None, collection_name: str = None):
        """
        Initialize the pipeline orchestrator.

        Args:
            chunk_size: Size of text chunks
            chunk_overlap: Overlap between chunks
            cohere_model: Cohere embedding model to use
            collection_name: Qdrant collection name
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.cohere_model = cohere_model or Config.COHERE_MODEL
        self.collection_name = collection_name or Config.QDRANT_COLLECTION_NAME

        # Validate configuration
        Config.validate()

    def run_pipeline(self, urls: List[str], progress_callback=None) -> Dict[str, Any]:
        """
        Run the complete pipeline from URL ingestion to vector storage.

        Args:
            urls: List of URLs to process
            progress_callback: Optional callback function to report progress

        Returns:
            Dictionary with pipeline results
        """
        start_time = time.time()
        total_steps = 5  # Total number of major pipeline steps
        current_step = 0

        pipeline_state = PipelineState(status='initialized')

        try:
            # Validate inputs
            if not urls or not isinstance(urls, list):
                raise ValueError("URLs must be a non-empty list")

            # Update pipeline state
            pipeline_state.status = 'ingesting'
            current_step = 1
            pipeline_state.progress = (current_step / total_steps) * 100
            if progress_callback:
                progress_callback(pipeline_state)

            # Step 1: Ingest URLs
            logger.info(f"Starting ingestion for {len(urls)} URLs")
            try:
                ingestion_results = ingest_urls(urls)
                successful_results = [r for r in ingestion_results if r.get('status') == 'success']

                pipeline_state.processed_urls = [r['url'] for r in successful_results]
                pipeline_state.failed_urls = [
                    {'url': r['url'], 'error': r.get('error_message', 'Unknown error')}
                    for r in ingestion_results if r.get('status') == 'failed'
                ]

                if not successful_results:
                    logger.warning("No URLs were successfully ingested")
                    # Continue with empty list to allow for proper error reporting

                if progress_callback:
                    progress_callback(pipeline_state)

            except Exception as e:
                logger.error(f"Error during URL ingestion: {str(e)}")
                pipeline_state.status = 'failed'
                pipeline_state.failed_urls.append({
                    'url': 'ingestion',
                    'error': f'Ingestion error: {str(e)}'
                })
                raise

            # Update pipeline state
            pipeline_state.status = 'cleaning'
            current_step = 2
            pipeline_state.progress = (current_step / total_steps) * 100
            if progress_callback:
                progress_callback(pipeline_state)

            # Step 2: Clean content
            logger.info(f"Cleaning content from {len(successful_results)} URLs")
            cleaned_contents = []
            try:
                for idx, result in enumerate(successful_results):
                    if result.get('content'):
                        cleaned_content = clean_content(result['content'])
                        cleaned_contents.append({
                            'url': result['url'],
                            'title': result.get('title', ''),
                            'content': cleaned_content,
                            'original_length': result.get('content_length', 0),
                            'cleaned_length': len(cleaned_content)
                        })

                    # Update progress within this step
                    if len(successful_results) > 0:
                        step_progress = (idx + 1) / len(successful_results) * 20  # 20% of total for this step
                        pipeline_state.progress = (current_step - 1) * (100 / total_steps) + step_progress
                        if progress_callback:
                            progress_callback(pipeline_state)

            except Exception as e:
                logger.error(f"Error during content cleaning: {str(e)}")
                pipeline_state.status = 'failed'
                pipeline_state.failed_urls.append({
                    'url': 'cleaning',
                    'error': f'Cleaning error: {str(e)}'
                })
                raise

            # Update pipeline state
            pipeline_state.status = 'chunking'
            current_step = 3
            pipeline_state.progress = (current_step / total_steps) * 100
            if progress_callback:
                progress_callback(pipeline_state)

            # Step 3: Chunk content
            logger.info("Chunking content")
            all_chunks = []
            try:
                for idx, item in enumerate(cleaned_contents):
                    chunks = chunk_content(
                        content=item['content'],
                        url=item['url'],
                        chunk_size=self.chunk_size,
                        chunk_overlap=self.chunk_overlap,
                        section=item.get('title', '')
                    )
                    all_chunks.extend(chunks)

                    # Update progress within this step
                    if len(cleaned_contents) > 0:
                        step_progress = (idx + 1) / len(cleaned_contents) * 20  # 20% of total for this step
                        pipeline_state.progress = (current_step - 1) * (100 / total_steps) + step_progress
                        if progress_callback:
                            progress_callback(pipeline_state)

            except Exception as e:
                logger.error(f"Error during content chunking: {str(e)}")
                pipeline_state.status = 'failed'
                pipeline_state.failed_urls.append({
                    'url': 'chunking',
                    'error': f'Chunking error: {str(e)}'
                })
                raise

            # Update pipeline state
            pipeline_state.total_chunks = len(all_chunks)
            pipeline_state.status = 'embedding'
            current_step = 4
            pipeline_state.progress = (current_step / total_steps) * 100
            if progress_callback:
                progress_callback(pipeline_state)

            # Step 4: Generate embeddings
            logger.info(f"Generating embeddings for {len(all_chunks)} chunks")
            chunks_with_embeddings = []
            try:
                chunks_with_embeddings = generate_embeddings_for_chunks(
                    all_chunks,
                    model=self.cohere_model
                )

                # Check if any embeddings failed to generate
                successful_embeddings = [c for c in chunks_with_embeddings if c.get('embedding')]
                failed_embeddings_count = len(all_chunks) - len(successful_embeddings)
                if failed_embeddings_count > 0:
                    logger.warning(f"{failed_embeddings_count} chunks failed to generate embeddings")

            except Exception as e:
                logger.error(f"Error during embedding generation: {str(e)}")
                pipeline_state.status = 'failed'
                pipeline_state.failed_urls.append({
                    'url': 'embedding',
                    'error': f'Embedding error: {str(e)}'
                })
                raise

            # Update pipeline state
            pipeline_state.status = 'storing'
            current_step = 5
            pipeline_state.progress = (current_step / total_steps) * 100
            if progress_callback:
                progress_callback(pipeline_state)

            # Step 5: Store embeddings in Qdrant
            logger.info("Storing embeddings in Qdrant")
            stored_count = 0
            try:
                stored_count = store_embeddings_in_qdrant(
                    chunks_with_embeddings,
                    collection_name=self.collection_name
                )

            except Exception as e:
                logger.error(f"Error during storage: {str(e)}")
                pipeline_state.status = 'failed'
                pipeline_state.failed_urls.append({
                    'url': 'storage',
                    'error': f'Storage error: {str(e)}'
                })
                raise

            # Update pipeline state
            pipeline_state.status = 'completed'
            pipeline_state.progress = 100.0
            if progress_callback:
                progress_callback(pipeline_state)

            # Calculate final metrics
            total_time = time.time() - start_time
            processed_count = len(pipeline_state.processed_urls)

            result = {
                'status': 'success',
                'processed_count': processed_count,
                'stored_count': stored_count,
                'total_chunks': len(all_chunks),
                'failed_count': len(pipeline_state.failed_urls),
                'total_time': total_time,
                'errors': pipeline_state.failed_urls,
                'pipeline_state': pipeline_state.to_dict()
            }

            logger.info(f"Pipeline completed successfully: {processed_count} URLs processed, {stored_count} chunks stored")
            return result

        except Exception as e:
            logger.error(f"Pipeline failed: {str(e)}")
            if pipeline_state.status != 'failed':  # Only update if not already marked as failed
                pipeline_state.status = 'failed'
                pipeline_state.progress = 0.0
                pipeline_state.failed_urls.append({
                    'url': 'pipeline',
                    'error': str(e)
                })

            result = {
                'status': 'failed',
                'processed_count': len(pipeline_state.processed_urls),
                'stored_count': 0,
                'total_chunks': 0,
                'failed_count': len(pipeline_state.failed_urls),
                'errors': pipeline_state.failed_urls,
                'pipeline_state': pipeline_state.to_dict()
            }

            return result

    def run_pipeline_with_progress_tracking(self, urls: List[str]) -> Dict[str, Any]:
        """
        Run the pipeline with detailed progress tracking.

        Args:
            urls: List of URLs to process

        Returns:
            Dictionary with pipeline results and progress tracking
        """
        logger.info(f"Starting pipeline with progress tracking for {len(urls)} URLs")
        progress_log = []

        def progress_callback(state):
            progress_entry = {
                'status': state.status,
                'timestamp': datetime.now().isoformat(),
                'processed_count': len(state.processed_urls),
                'failed_count': len(state.failed_urls),
                'total_chunks': state.total_chunks,
                'progress': state.progress
            }
            progress_log.append(progress_entry)
            logger.info(f"Pipeline progress: {state.status} ({state.progress:.1f}%) - "
                       f"Processed: {len(state.processed_urls)}, Failed: {len(state.failed_urls)}")

        result = self.run_pipeline(urls, progress_callback)
        result['progress_log'] = progress_log
        logger.info("Pipeline execution with progress tracking completed")
        return result


def run_pipeline_from_config(urls: List[str],
                           chunk_size: int = 1000,
                           chunk_overlap: int = 200,
                           cohere_model: str = None,
                           collection_name: str = None) -> Dict[str, Any]:
    """
    Run the complete pipeline using configuration parameters.

    Args:
        urls: List of URLs to process
        chunk_size: Size of text chunks
        chunk_overlap: Overlap between chunks
        cohere_model: Cohere embedding model to use
        collection_name: Qdrant collection name

    Returns:
        Dictionary with pipeline results
    """
    orchestrator = PipelineOrchestrator(
        chunk_size=chunk_size,
        chunk_overlap=chunk_overlap,
        cohere_model=cohere_model,
        collection_name=collection_name
    )

    return orchestrator.run_pipeline(urls)


def validate_pipeline_config() -> bool:
    """
    Validate that the pipeline configuration is correct.

    Returns:
        True if configuration is valid, False otherwise
    """
    try:
        Config.validate()
        return True
    except ValueError as e:
        logger.error(f"Pipeline configuration validation failed: {str(e)}")
        return False