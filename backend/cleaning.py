"""
Content cleaning module with BeautifulSoup
"""
import re
from typing import Dict, Any, List
import logging
from bs4 import BeautifulSoup, NavigableString, Comment

from errors import ContentProcessingError
from utils import clean_text


logger = logging.getLogger(__name__)


def clean_content(content: str, remove_code: bool = False, remove_links: bool = False) -> str:
    """
    Clean raw content by removing HTML elements and unnecessary text.

    Args:
        content: Raw content to clean
        remove_code: Whether to remove code blocks
        remove_links: Whether to remove links

    Returns:
        Cleaned text content
    """
    try:
        # If the content looks like HTML, parse it with BeautifulSoup
        if '<' in content and '>' in content:
            soup = BeautifulSoup(content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            # Remove comments
            for comment in soup.find_all(string=lambda text: isinstance(text, Comment)):
                comment.extract()

            # Remove code blocks if requested
            if remove_code:
                for code in soup(["code", "pre"]):
                    code.decompose()

            # Remove links if requested
            if remove_links:
                for link in soup.find_all('a'):
                    link.unwrap()  # Remove the link tag but keep the content

            # Get text content
            text = soup.get_text()
        else:
            # If it's already plain text, just clean it
            text = content

        # Clean up the text
        text = clean_text(text)

        # Remove extra newlines and normalize whitespace
        text = re.sub(r'\n\s*\n', '\n\n', text)  # Replace multiple newlines with max 2
        text = re.sub(r'[ \t]+', ' ', text)      # Replace multiple spaces/tabs with single space

        logger.debug(f"Cleaned content from {len(content)} to {len(text)} characters")

        return text

    except Exception as e:
        logger.error(f"Error cleaning content: {str(e)}")
        raise ContentProcessingError(f"Failed to clean content: {str(e)}")


def extract_headings_and_sections(content: str) -> List[Dict[str, Any]]:
    """
    Extract headings and sections from HTML content.

    Args:
        content: HTML content to extract headings from

    Returns:
        List of dictionaries containing heading text and level
    """
    try:
        soup = BeautifulSoup(content, 'html.parser')
        headings = []

        # Find all heading tags (h1, h2, h3, etc.)
        for heading in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6']):
            heading_text = clean_text(heading.get_text())
            if heading_text:  # Only add if there's actual text
                headings.append({
                    'level': int(heading.name[1]),  # Extract number from h1, h2, etc.
                    'text': heading_text,
                    'position': heading.sourceline if hasattr(heading, 'sourceline') else None
                })

        logger.debug(f"Extracted {len(headings)} headings from content")
        return headings

    except Exception as e:
        logger.error(f"Error extracting headings: {str(e)}")
        raise ContentProcessingError(f"Failed to extract headings: {str(e)}")


def remove_stop_words(text: str, stop_words: List[str] = None) -> str:
    """
    Remove common stop words from text (optional cleaning step).

    Args:
        text: Text to process
        stop_words: List of stop words to remove (default: common English stop words)

    Returns:
        Text with stop words removed
    """
    if stop_words is None:
        stop_words = [
            'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for',
            'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'being',
            'have', 'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could',
            'should', 'may', 'might', 'must', 'can', 'this', 'that', 'these', 'those'
        ]

    # Convert to lowercase and split into words
    words = text.split()
    filtered_words = [word for word in words if word.lower().strip('.,!?;:"()[]{}') not in stop_words]

    return ' '.join(filtered_words)


def extract_metadata(content: str) -> Dict[str, Any]:
    """
    Extract metadata from HTML content such as title, description, etc.

    Args:
        content: HTML content to extract metadata from

    Returns:
        Dictionary containing extracted metadata
    """
    try:
        soup = BeautifulSoup(content, 'html.parser')
        metadata = {}

        # Extract title
        title_tag = soup.find('title')
        if title_tag:
            metadata['title'] = clean_text(title_tag.get_text())

        # Extract meta description
        desc_tag = soup.find('meta', attrs={'name': 'description'})
        if desc_tag and desc_tag.get('content'):
            metadata['description'] = clean_text(desc_tag.get('content'))

        # Extract meta keywords
        keywords_tag = soup.find('meta', attrs={'name': 'keywords'})
        if keywords_tag and keywords_tag.get('content'):
            metadata['keywords'] = [clean_text(k.strip()) for k in keywords_tag.get('content').split(',')]

        # Extract Open Graph tags
        og_title = soup.find('meta', attrs={'property': 'og:title'})
        if og_title and og_title.get('content'):
            metadata['og_title'] = clean_text(og_title.get('content'))

        og_description = soup.find('meta', attrs={'property': 'og:description'})
        if og_description and og_description.get('content'):
            metadata['og_description'] = clean_text(og_description.get('content'))

        logger.debug(f"Extracted metadata: {list(metadata.keys())}")
        return metadata

    except Exception as e:
        logger.error(f"Error extracting metadata: {str(e)}")
        raise ContentProcessingError(f"Failed to extract metadata: {str(e)}")


def clean_for_embedding(text: str) -> str:
    """
    Clean text specifically for embedding generation, removing unnecessary elements
    while preserving semantic meaning.

    Args:
        text: Text to clean for embedding

    Returns:
        Cleaned text suitable for embedding
    """
    # Clean the text
    cleaned = clean_content(text)

    # Remove excessive punctuation while preserving sentence structure
    cleaned = re.sub(r'[!]{2,}', '!', cleaned)  # Multiple exclamation marks
    cleaned = re.sub(r'[?]{2,}', '?', cleaned)  # Multiple question marks
    cleaned = re.sub(r'[.]{3,}', '.', cleaned)  # Multiple periods (but keep ellipsis as single)

    # Ensure proper spacing around punctuation
    cleaned = re.sub(r'([.!?])\s*', r'\1 ', cleaned)

    # Clean up any remaining excessive whitespace
    cleaned = clean_text(cleaned)

    return cleaned