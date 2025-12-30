"""
Logging module with appropriate log levels
"""
import logging
import sys


def setup_logging(log_level: str = "INFO"):
    """
    Setup logging configuration with appropriate log levels.

    Args:
        log_level: The logging level as a string (DEBUG, INFO, WARNING, ERROR)
    """
    # Convert string level to logging constant
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)

    # Configure root logger
    logging.basicConfig(
        level=numeric_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('pipeline.log')
        ]
    )

    # Set specific log levels for external libraries to reduce noise
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("requests").setLevel(logging.WARNING)
    logging.getLogger("cohere").setLevel(logging.INFO)
    logging.getLogger("qdrant_client").setLevel(logging.INFO)