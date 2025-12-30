"""
Entity definitions for the pipeline
"""
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from datetime import datetime


@dataclass
class URLProcessingResult:
    """
    Entity representing the result of processing a URL.

    Based on the data model specification:
    - url: The processed URL (string, required)
    - status: Processing status (string, required: "success", "failed", "skipped")
    - title: Page title (string, optional)
    - content_length: Length of extracted content (number, optional)
    - error_message: Error details if processing failed (string, optional)
    - processed_at: When processing was completed (ISO string, required)
    """
    url: str
    status: str  # "success", "failed", "skipped"
    title: Optional[str] = None
    content_length: Optional[int] = None
    error_message: Optional[str] = None
    processed_at: Optional[datetime] = None

    def __post_init__(self):
        if self.processed_at is None:
            self.processed_at = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            'url': self.url,
            'status': self.status,
            'title': self.title,
            'content_length': self.content_length,
            'error_message': self.error_message,
            'processed_at': self.processed_at.isoformat() if self.processed_at else None
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'URLProcessingResult':
        """Create instance from dictionary."""
        processed_at = datetime.fromisoformat(data['processed_at']) if data.get('processed_at') else None
        return cls(
            url=data['url'],
            status=data['status'],
            title=data.get('title'),
            content_length=data.get('content_length'),
            error_message=data.get('error_message'),
            processed_at=processed_at
        )


@dataclass
class ContentChunk:
    """
    Entity representing a chunk of content with metadata.

    Based on the data model specification:
    - id: Unique identifier for the chunk (string, required)
    - url: Source URL of the content (string, required)
    - section: Section/heading from the source (string, optional)
    - content: Cleaned text content of the chunk (string, required)
    - metadata: Additional metadata dictionary (object, optional)
    - embedding: Vector representation of the content (array of numbers, required)
    - created_at: Timestamp when chunk was created (ISO string, required)
    """
    id: str
    url: str
    content: str
    section: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None
    embedding: Optional[List[float]] = None
    created_at: Optional[datetime] = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()
        if self.metadata is None:
            self.metadata = {}

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            'id': self.id,
            'url': self.url,
            'section': self.section,
            'content': self.content,
            'metadata': self.metadata,
            'embedding': self.embedding,
            'created_at': self.created_at.isoformat() if self.created_at else None
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ContentChunk':
        """Create instance from dictionary."""
        created_at = datetime.fromisoformat(data['created_at']) if data.get('created_at') else None
        return cls(
            id=data['id'],
            url=data['url'],
            section=data.get('section'),
            content=data['content'],
            metadata=data.get('metadata', {}),
            embedding=data.get('embedding'),
            created_at=created_at
        )


@dataclass
class PipelineState:
    """
    Entity representing the current state of the pipeline.

    Based on the data model specification:
    - status: Current stage of the pipeline (string, required)
    - processed_urls: List of URLs successfully processed (array of strings, optional)
    - failed_urls: List of URLs that failed processing (array of objects with url and error, optional)
    - total_chunks: Total number of content chunks created (number, optional)
    - progress: Progress percentage (number, 0-100, optional)
    - timestamp: When the state was recorded (ISO string, required)
    """
    status: str  # initialized, ingesting, cleaning, chunking, embedding, storing, completed, failed
    processed_urls: Optional[List[str]] = None
    failed_urls: Optional[List[Dict[str, str]]] = None  # List of {url: str, error: str}
    total_chunks: Optional[int] = None
    progress: Optional[float] = None
    timestamp: Optional[datetime] = None

    def __post_init__(self):
        if self.processed_urls is None:
            self.processed_urls = []
        if self.failed_urls is None:
            self.failed_urls = []
        if self.timestamp is None:
            self.timestamp = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            'status': self.status,
            'processed_urls': self.processed_urls,
            'failed_urls': self.failed_urls,
            'total_chunks': self.total_chunks,
            'progress': self.progress,
            'timestamp': self.timestamp.isoformat() if self.timestamp else None
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'PipelineState':
        """Create instance from dictionary."""
        timestamp = datetime.fromisoformat(data['timestamp']) if data.get('timestamp') else None
        return cls(
            status=data['status'],
            processed_urls=data.get('processed_urls', []),
            failed_urls=data.get('failed_urls', []),
            total_chunks=data.get('total_chunks'),
            progress=data.get('progress'),
            timestamp=timestamp
        )


@dataclass
class EmbeddingResult:
    """
    Entity representing the result of embedding generation.

    Based on the data model specification:
    - chunk_id: ID of the content chunk (string, required)
    - embedding_vector: The numerical embedding vector (array of numbers, required)
    - text_length: Length of the original text (number, required)
    - processing_time: Time taken to generate embedding in milliseconds (number, required)
    """
    chunk_id: str
    embedding_vector: List[float]
    text_length: int
    processing_time: float  # in milliseconds

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            'chunk_id': self.chunk_id,
            'embedding_vector': self.embedding_vector,
            'text_length': self.text_length,
            'processing_time': self.processing_time
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'EmbeddingResult':
        """Create instance from dictionary."""
        return cls(
            chunk_id=data['chunk_id'],
            embedding_vector=data['embedding_vector'],
            text_length=data['text_length'],
            processing_time=data['processing_time']
        )