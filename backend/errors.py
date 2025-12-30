"""
Error handling module with custom exceptions
"""


class PipelineError(Exception):
    """Base exception for pipeline errors."""
    pass


class ConfigurationError(PipelineError):
    """Raised when there are configuration issues."""
    pass


class IngestionError(PipelineError):
    """Raised when URL ingestion fails."""
    pass


class ContentProcessingError(PipelineError):
    """Raised when content cleaning or chunking fails."""
    pass


class EmbeddingError(PipelineError):
    """Raised when embedding generation fails."""
    pass


class StorageError(PipelineError):
    """Raised when storage operations fail."""
    pass


class NetworkError(PipelineError):
    """Raised when network-related operations fail."""
    pass


class ValidationError(PipelineError):
    """Raised when input validation fails."""
    pass