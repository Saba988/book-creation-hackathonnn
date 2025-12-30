"""
Content chunking module with configurable size/overlap
"""
from typing import List, Dict, Any
import logging
from utils import generate_chunk_id, clean_text


logger = logging.getLogger(__name__)


def chunk_content(content: str, url: str, chunk_size: int = 1000, chunk_overlap: int = 200,
                 include_headings: bool = True, section: str = None) -> List[Dict[str, Any]]:
    """
    Chunk content into segments with configurable size and overlap.

    Args:
        content: Content to chunk
        url: Source URL for the content
        chunk_size: Size of each chunk in characters
        chunk_overlap: Overlap between chunks in characters
        include_headings: Whether to try to preserve headings with chunks
        section: Section/heading from the source (optional)

    Returns:
        List of chunk dictionaries with content, metadata, and IDs
    """
    if chunk_size <= 0:
        raise ValueError("Chunk size must be greater than 0")

    if chunk_overlap >= chunk_size:
        raise ValueError("Chunk overlap must be less than chunk size")

    # Clean the content first
    content = clean_text(content)

    chunks = []
    start_idx = 0
    chunk_idx = 0

    while start_idx < len(content):
        # Calculate end index
        end_idx = start_idx + chunk_size

        # If this is the last chunk, include the rest of the content
        if end_idx >= len(content):
            end_idx = len(content)
        else:
            # Try to break at a sentence boundary to avoid cutting sentences
            # Find the nearest sentence boundary before the end
            chunk_content = content[start_idx:end_idx]
            last_sentence_idx = chunk_content.rfind('. ')

            if last_sentence_idx > chunk_size // 2:  # Only if it's reasonably far into the chunk
                end_idx = start_idx + last_sentence_idx + 2  # +2 to include the '. '

        # Extract the chunk text
        chunk_text = content[start_idx:end_idx]

        # Generate chunk ID
        chunk_id = generate_chunk_id(url, chunk_idx)

        # Create chunk dictionary with rich metadata
        chunk = {
            'id': chunk_id,
            'url': url,
            'section': section or f"chunk_{chunk_idx}",
            'content': chunk_text,
            'metadata': {
                'position': chunk_idx,
                'start_idx': start_idx,
                'end_idx': end_idx,
                'length': len(chunk_text),
                'url': url,
                'section': section,
                'chunk_index': chunk_idx,
                'total_chunks': 0  # Will be updated after all chunks are created
            },
            'created_at': f"{__import__('datetime').datetime.now().isoformat()}"
        }

        chunks.append(chunk)

        # Move to the next chunk, accounting for overlap
        if end_idx >= len(content):
            # This was the last chunk, exit
            break

        start_idx = end_idx - chunk_overlap
        chunk_idx += 1

    # Update total chunks in metadata
    for i, chunk in enumerate(chunks):
        chunks[i]['metadata']['total_chunks'] = len(chunks)

    logger.info(f"Chunked content from {url} into {len(chunks)} chunks (size: {chunk_size}, overlap: {chunk_overlap})")
    return chunks


def chunk_with_headings(content: str, url: str, headings: List[Dict[str, Any]],
                       chunk_size: int = 1000, chunk_overlap: int = 200) -> List[Dict[str, Any]]:
    """
    Chunk content while preserving heading context.

    Args:
        content: Content to chunk
        url: Source URL for the content
        headings: List of headings from the content
        chunk_size: Size of each chunk in characters
        chunk_overlap: Overlap between chunks in characters

    Returns:
        List of chunk dictionaries with content, metadata, and headings context
    """
    # For now, use the basic chunking method but with special handling for headings
    # In a more sophisticated implementation, we would align chunks with headings

    chunks = chunk_content(content, url, chunk_size, chunk_overlap)

    # Add heading context to each chunk if possible
    for chunk in chunks:
        # Find the most relevant heading for this chunk based on position
        chunk_start = chunk['metadata']['start_idx']

        # Find the heading that appears before or closest to this chunk
        relevant_heading = None
        max_distance = float('inf')

        for heading in headings:
            heading_pos = heading.get('position', 0)
            distance = abs(heading_pos - chunk_start)

            # Consider headings within a reasonable distance and before the chunk
            if heading_pos <= chunk_start and distance < max_distance:
                max_distance = distance
                relevant_heading = heading

        if relevant_heading:
            chunk['section'] = relevant_heading['text']
            if 'metadata' not in chunk:
                chunk['metadata'] = {}
            chunk['metadata']['heading'] = relevant_heading

    return chunks


def adaptive_chunking(content: str, url: str,
                     min_chunk_size: int = 500,
                     max_chunk_size: int = 2000,
                     target_chunk_size: int = 1000,
                     chunk_overlap: int = 200) -> List[Dict[str, Any]]:
    """
    Adaptive chunking that adjusts chunk sizes based on content structure.

    Args:
        content: Content to chunk
        url: Source URL for the content
        min_chunk_size: Minimum chunk size
        max_chunk_size: Maximum chunk size
        target_chunk_size: Target chunk size
        chunk_overlap: Overlap between chunks

    Returns:
        List of chunk dictionaries
    """
    # Clean the content first
    content = clean_text(content)

    # For adaptive chunking, we could use semantic boundaries, paragraphs, or other
    # structural elements to determine better chunk boundaries
    # For now, implement paragraph-based chunking as an example of adaptive approach

    paragraphs = content.split('\n\n')
    chunks = []
    current_chunk = ""
    chunk_idx = 0

    for para in paragraphs:
        # If adding this paragraph would exceed the target size
        if len(current_chunk) + len(para) > target_chunk_size and current_chunk:
            # Finalize current chunk
            chunk_id = generate_chunk_id(url, chunk_idx)
            chunk = {
                'id': chunk_id,
                'url': url,
                'section': f"paragraph_chunk_{chunk_idx}",
                'content': clean_text(current_chunk),
                'metadata': {
                    'position': chunk_idx,
                    'length': len(current_chunk),
                    'chunking_method': 'paragraph'
                },
                'created_at': f"{__import__('datetime').datetime.now().isoformat()}"
            }
            chunks.append(chunk)
            chunk_idx += 1

            # Start new chunk - include overlap if possible
            if chunk_overlap > 0 and len(para) > chunk_overlap:
                # Use end of current chunk as beginning of next chunk
                overlap_start = max(0, len(current_chunk) - chunk_overlap)
                current_chunk = current_chunk[overlap_start:] + para
            else:
                current_chunk = para
        else:
            current_chunk += "\n\n" + para if current_chunk else para

    # Add the last chunk if there's remaining content
    if current_chunk.strip():
        chunk_id = generate_chunk_id(url, chunk_idx)
        chunk = {
            'id': chunk_id,
            'url': url,
            'section': f"paragraph_chunk_{chunk_idx}",
            'content': clean_text(current_chunk),
            'metadata': {
                'position': chunk_idx,
                'length': len(current_chunk),
                'chunking_method': 'paragraph'
            },
            'created_at': f"{__import__('datetime').datetime.now().isoformat()}"
        }
        chunks.append(chunk)

    logger.info(f"Adaptive chunking created {len(chunks)} chunks from {url}")
    return chunks


def validate_chunk_parameters(chunk_size: int, chunk_overlap: int, min_size: int = 100, max_size: int = 10000) -> bool:
    """
    Validate chunking parameters.

    Args:
        chunk_size: The chunk size to validate
        chunk_overlap: The chunk overlap to validate
        min_size: Minimum allowed chunk size
        max_size: Maximum allowed chunk size

    Returns:
        True if parameters are valid, False otherwise
    """
    if not (min_size <= chunk_size <= max_size):
        logger.error(f"Chunk size {chunk_size} is outside valid range [{min_size}, {max_size}]")
        return False

    if chunk_overlap >= chunk_size:
        logger.error(f"Chunk overlap {chunk_overlap} must be less than chunk size {chunk_size}")
        return False

    if chunk_overlap < 0:
        logger.error(f"Chunk overlap {chunk_overlap} cannot be negative")
        return False

    return True