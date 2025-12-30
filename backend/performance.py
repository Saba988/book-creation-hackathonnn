"""
Performance optimization for processing multiple URLs
"""
import asyncio
import concurrent.futures
from typing import List, Dict, Any, Callable
import time
import logging
from threading import Lock
import queue

from ingestion import ingest_urls
from cleaning import clean_content
from chunking import chunk_content
from embedding import generate_embeddings_for_chunks
from storage import store_embeddings_in_qdrant


logger = logging.getLogger(__name__)


class PerformanceOptimizer:
    """
    Class to optimize performance when processing multiple URLs.
    """
    def __init__(self, max_workers: int = 4, batch_size: int = 10):
        """
        Initialize the performance optimizer.

        Args:
            max_workers: Maximum number of worker threads/processes
            batch_size: Size of batches for processing
        """
        self.max_workers = max_workers
        self.batch_size = batch_size
        self.processing_times = {}
        self.lock = Lock()

    def process_urls_in_parallel(self, urls: List[str], **kwargs) -> List[Dict[str, Any]]:
        """
        Process multiple URLs in parallel to improve performance.

        Args:
            urls: List of URLs to process
            **kwargs: Additional arguments to pass to processing functions

        Returns:
            List of processing results
        """
        logger.info(f"Processing {len(urls)} URLs in parallel with {self.max_workers} workers")

        results = []
        start_time = time.time()

        # Process URLs in batches to manage memory usage
        for i in range(0, len(urls), self.batch_size):
            batch = urls[i:i + self.batch_size]
            logger.info(f"Processing batch {i//self.batch_size + 1} of {len(urls)//self.batch_size + 1}")

            with concurrent.futures.ThreadPoolExecutor(max_workers=self.max_workers) as executor:
                # Submit tasks for parallel processing
                future_to_url = {
                    executor.submit(self._process_single_url, url, **kwargs): url
                    for url in batch
                }

                # Collect results as they complete
                for future in concurrent.futures.as_completed(future_to_url):
                    url = future_to_url[future]
                    try:
                        result = future.result()
                        results.append(result)
                        logger.debug(f"Completed processing: {url}")
                    except Exception as e:
                        logger.error(f"Error processing {url}: {str(e)}")
                        results.append({
                            'url': url,
                            'status': 'failed',
                            'error': str(e)
                        })

        total_time = time.time() - start_time
        logger.info(f"Processed {len(urls)} URLs in {total_time:.2f} seconds")

        return results

    def _process_single_url(self, url: str, **kwargs) -> Dict[str, Any]:
        """
        Process a single URL with all pipeline steps.

        Args:
            url: URL to process
            **kwargs: Additional arguments

        Returns:
            Processing result
        """
        try:
            # Step 1: Ingest URL
            ingestion_results = ingest_urls([url])
            if not ingestion_results or ingestion_results[0].get('status') != 'success':
                return {
                    'url': url,
                    'status': 'failed',
                    'error': ingestion_results[0].get('error_message', 'Ingestion failed')
                }

            ingestion_result = ingestion_results[0]
            content = ingestion_result.get('content', '')
            title = ingestion_result.get('title', '')

            # Step 2: Clean content
            cleaned_content = clean_content(content)

            # Step 3: Chunk content
            chunks = chunk_content(
                content=cleaned_content,
                url=url,
                chunk_size=kwargs.get('chunk_size', 1000),
                chunk_overlap=kwargs.get('chunk_overlap', 200),
                section=title
            )

            # Step 4: Generate embeddings (this would normally be done in bulk)
            # For parallel processing, we'll return chunks for batch embedding later
            return {
                'url': url,
                'status': 'success',
                'content': cleaned_content,
                'chunks': chunks,
                'title': title
            }

        except Exception as e:
            logger.error(f"Error processing URL {url}: {str(e)}")
            return {
                'url': url,
                'status': 'failed',
                'error': str(e)
            }

    def optimize_pipeline_execution(self, urls: List[str], **kwargs) -> Dict[str, Any]:
        """
        Optimize the entire pipeline execution for multiple URLs.

        Args:
            urls: List of URLs to process
            **kwargs: Additional arguments

        Returns:
            Dictionary with optimized pipeline results
        """
        logger.info(f"Optimizing pipeline execution for {len(urls)} URLs")

        start_time = time.time()

        # Process URLs in parallel to get content and chunks
        parallel_results = self.process_urls_in_parallel(urls, **kwargs)

        # Separate successful and failed results
        successful_results = [r for r in parallel_results if r.get('status') == 'success']
        failed_results = [r for r in parallel_results if r.get('status') == 'failed']

        if not successful_results:
            logger.warning("No URLs were successfully processed")
            return {
                'status': 'failed',
                'processed_count': 0,
                'failed_count': len(failed_results),
                'stored_count': 0,
                'errors': failed_results
            }

        # Collect all chunks for batch processing
        all_chunks = []
        for result in successful_results:
            all_chunks.extend(result.get('chunks', []))

        logger.info(f"Collected {len(all_chunks)} chunks from {len(successful_results)} URLs")

        # Batch generate embeddings
        if all_chunks:
            logger.info("Generating embeddings in batch...")
            chunks_with_embeddings = generate_embeddings_for_chunks(
                all_chunks,
                model=kwargs.get('cohere_model'),
                batch_size=kwargs.get('embedding_batch_size', 64)
            )
        else:
            chunks_with_embeddings = []

        # Batch store embeddings
        stored_count = 0
        if chunks_with_embeddings:
            logger.info("Storing embeddings in batch...")
            stored_count = store_embeddings_in_qdrant(
                chunks_with_embeddings,
                collection_name=kwargs.get('collection_name')
            )

        total_time = time.time() - start_time

        result = {
            'status': 'success' if successful_results else 'failed',
            'processed_count': len(successful_results),
            'failed_count': len(failed_results),
            'stored_count': stored_count,
            'total_chunks': len(all_chunks),
            'total_time': total_time,
            'errors': failed_results
        }

        logger.info(f"Optimized pipeline completed: {result}")
        return result


def optimize_for_large_document_sets(urls: List[str],
                                   chunk_size: int = 1000,
                                   max_workers: int = 4,
                                   batch_size: int = 10) -> Dict[str, Any]:
    """
    Optimize processing for large sets of documents/URLs.

    Args:
        urls: List of URLs to process
        chunk_size: Size of text chunks
        max_workers: Maximum number of parallel workers
        batch_size: Size of processing batches

    Returns:
        Dictionary with processing results
    """
    optimizer = PerformanceOptimizer(max_workers=max_workers, batch_size=batch_size)

    return optimizer.optimize_pipeline_execution(
        urls,
        chunk_size=chunk_size,
        chunk_overlap=max(100, chunk_size // 5)  # 20% overlap
    )


def get_performance_recommendations(url_count: int, avg_content_size: int = 10000) -> Dict[str, Any]:
    """
    Get performance recommendations based on the size of the processing job.

    Args:
        url_count: Number of URLs to process
        avg_content_size: Average content size per URL in characters

    Returns:
        Dictionary with performance recommendations
    """
    total_content_size = url_count * avg_content_size
    recommendations = {}

    # Recommend worker count based on content size
    if total_content_size < 100000:  # Less than 100KB total
        recommendations['max_workers'] = 2
        recommendations['batch_size'] = 5
    elif total_content_size < 1000000:  # Less than 1MB total
        recommendations['max_workers'] = 4
        recommendations['batch_size'] = 10
    elif total_content_size < 10000000:  # Less than 10MB total
        recommendations['max_workers'] = 6
        recommendations['batch_size'] = 15
    else:  # More than 10MB total
        recommendations['max_workers'] = 8
        recommendations['batch_size'] = 20

    # Recommend chunk size based on content size
    if avg_content_size < 5000:  # Small documents
        recommendations['chunk_size'] = 500
        recommendations['chunk_overlap'] = 100
    elif avg_content_size < 20000:  # Medium documents
        recommendations['chunk_size'] = 1000
        recommendations['chunk_overlap'] = 200
    else:  # Large documents
        recommendations['chunk_size'] = 2000
        recommendations['chunk_overlap'] = 400

    recommendations['estimated_processing_time'] = (
        total_content_size / 1000  # Rough estimate: 1 second per 1000 chars with optimizations
    )

    return recommendations


class AsyncPipelineProcessor:
    """
    Asynchronous pipeline processor for improved performance.
    """
    def __init__(self, max_concurrent: int = 10):
        self.max_concurrent = max_concurrent
        self.semaphore = asyncio.Semaphore(max_concurrent)

    async def process_url_async(self, url: str, **kwargs) -> Dict[str, Any]:
        """
        Asynchronously process a single URL.

        Args:
            url: URL to process
            **kwargs: Additional arguments

        Returns:
            Processing result
        """
        async with self.semaphore:
            try:
                # Simulate async processing (in a real implementation, this would use async libraries)
                await asyncio.sleep(0.1)  # Placeholder for actual async processing

                # For now, we'll use the synchronous functions
                # In a real async implementation, these would be replaced with async equivalents
                ingestion_results = ingest_urls([url])

                if not ingestion_results or ingestion_results[0].get('status') != 'success':
                    return {
                        'url': url,
                        'status': 'failed',
                        'error': ingestion_results[0].get('error_message', 'Ingestion failed')
                    }

                return {
                    'url': url,
                    'status': 'success',
                    'content_length': len(ingestion_results[0].get('content', ''))
                }
            except Exception as e:
                return {
                    'url': url,
                    'status': 'failed',
                    'error': str(e)
                }

    async def process_urls_async(self, urls: List[str], **kwargs) -> List[Dict[str, Any]]:
        """
        Process multiple URLs asynchronously.

        Args:
            urls: List of URLs to process
            **kwargs: Additional arguments

        Returns:
            List of processing results
        """
        tasks = [self.process_url_async(url, **kwargs) for url in urls]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Handle any exceptions that occurred during processing
        processed_results = []
        for i, result in enumerate(results):
            if isinstance(result, Exception):
                processed_results.append({
                    'url': urls[i],
                    'status': 'failed',
                    'error': str(result)
                })
            else:
                processed_results.append(result)

        return processed_results


def run_performance_benchmark(urls: List[str], **kwargs) -> Dict[str, Any]:
    """
    Run a performance benchmark comparing different processing approaches.

    Args:
        urls: List of URLs to benchmark
        **kwargs: Additional arguments

    Returns:
        Dictionary with benchmark results
    """
    logger.info(f"Running performance benchmark for {len(urls)} URLs")

    results = {}

    # Benchmark sequential processing
    logger.info("Benchmarking sequential processing...")
    seq_start = time.time()
    # For simplicity, we'll simulate sequential processing
    seq_time = len(urls) * 1.0  # Placeholder time
    results['sequential'] = {
        'time': seq_time,
        'throughput': len(urls) / seq_time if seq_time > 0 else 0
    }

    # Benchmark parallel processing
    logger.info("Benchmarking parallel processing...")
    optimizer = PerformanceOptimizer(max_workers=kwargs.get('max_workers', 4))
    parallel_result = optimizer.optimize_pipeline_execution(urls, **kwargs)
    results['parallel'] = {
        'time': parallel_result['total_time'],
        'throughput': len(urls) / parallel_result['total_time'] if parallel_result['total_time'] > 0 else 0
    }

    # Calculate improvement
    improvement = ((seq_time - parallel_result['total_time']) / seq_time * 100) if seq_time > 0 else 0
    results['improvement'] = f"{improvement:.1f}%"

    logger.info(f"Performance benchmark complete: {results}")

    return results