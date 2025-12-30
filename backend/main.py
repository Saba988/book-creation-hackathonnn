#!/usr/bin/env python3
"""
URL Ingestion and Embedding Pipeline for RAG System

This script implements a complete pipeline for RAG (Retrieval Augmented Generation)
that handles URL ingestion, content processing, embedding generation, and vector storage.
"""

import argparse
import logging
from typing import List, Dict, Any, Optional

# Import implemented modules
from config import Config
from logging_config import setup_logging
from errors import PipelineError
from pipeline import PipelineOrchestrator
from shutdown_handler import run_pipeline_with_shutdown_handling


def main():
    """Main execution function that orchestrates the complete pipeline."""
    parser = argparse.ArgumentParser(description='URL Ingestion and Embedding Pipeline')
    parser.add_argument('--urls', nargs='+', required=True,
                        help='List of URLs to process')
    parser.add_argument('--chunk-size', type=int, default=1000,
                        help='Size of text chunks (default: 1000)')
    parser.add_argument('--chunk-overlap', type=int, default=200,
                        help='Overlap between chunks (default: 200)')
    parser.add_argument('--model', default='embed-multilingual-v3.0',
                        help='Cohere embedding model to use')
    parser.add_argument('--collection', default='rag_content',
                        help='Qdrant collection name')
    parser.add_argument('--log-level', default='INFO',
                        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                        help='Logging level')

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.log_level)
    logger = logging.getLogger(__name__)

    logger.info("Starting URL ingestion and embedding pipeline with graceful shutdown handling")

    try:
        # Validate configuration
        Config.validate()
        logger.debug("Configuration validated successfully")

        # Run the pipeline with shutdown handling
        result = run_pipeline_with_shutdown_handling(
            args.urls,
            chunk_size=args.chunk_size,
            chunk_overlap=args.chunk_overlap,
            cohere_model=args.model,
            collection_name=args.collection
        )

        if result['status'] == 'success':
            logger.info(f"Pipeline completed successfully.")
            logger.info(f"  - URLs processed: {result['processed_count']}")
            logger.info(f"  - Chunks stored: {result['stored_count']}")
            logger.info(f"  - Total chunks created: {result['total_chunks']}")
            logger.info(f"  - Total time: {result['total_time']:.2f} seconds")

            if result.get('errors'):
                logger.warning(f"Encountered {len(result['errors'])} errors during processing:")
                for error in result['errors']:
                    logger.warning(f"  - {error['url']}: {error['error']}")
            else:
                logger.info("No errors encountered during processing.")
        elif result['status'] == 'cancelled':
            logger.info("Pipeline was cancelled (likely due to shutdown request)")
        else:
            logger.error(f"Pipeline failed: {result.get('error', 'Unknown error')}")

        return result

    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        return {'status': 'cancelled', 'error': 'Interrupted by user'}
    except Exception as e:
        logger.error(f"Pipeline failed with error: {str(e)}")
        raise


if __name__ == "__main__":
    main()