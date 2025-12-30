"""
URL ingestion module with requests
"""
import requests
from typing import List, Dict, Any, Optional
from urllib.parse import urljoin, urlparse
import time
import logging
from bs4 import BeautifulSoup

from errors import IngestionError, NetworkError
from utils import validate_url


logger = logging.getLogger(__name__)


def ingest_urls(urls: List[str], timeout: int = 30, max_retries: int = 3) -> List[Dict[str, Any]]:
    """
    Ingest content from a list of URLs with enhanced error handling.

    Args:
        urls: List of URLs to ingest
        timeout: Request timeout in seconds
        max_retries: Maximum number of retries for failed requests

    Returns:
        List of dictionaries containing URL, content, title, and metadata
    """
    results = []

    for url in urls:
        success = False
        attempt = 0
        error_message = ""

        while not success and attempt < max_retries:
            try:
                if not validate_url(url):
                    raise IngestionError(f"Invalid URL format: {url}")

                logger.info(f"Ingesting content from {url} (attempt {attempt + 1})")

                # Make request with appropriate headers
                headers = {
                    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
                }

                response = requests.get(url, headers=headers, timeout=timeout)
                response.raise_for_status()  # Raise an exception for bad status codes

                # Parse the HTML content
                soup = BeautifulSoup(response.content, 'html.parser')

                # Extract title
                title_tag = soup.find('title')
                title = title_tag.get_text().strip() if title_tag else "No Title"

                # Get the main content - try to get the most relevant content
                # Prioritize main content areas
                content_element = (
                    soup.find('main') or
                    soup.find('article') or
                    soup.find('div', class_='content') or
                    soup.find('div', id='content') or
                    soup.find('body')
                )

                content = content_element.get_text(separator=' ') if content_element else soup.get_text()

                # Clean up the content
                content = ' '.join(content.split())  # Normalize whitespace

                result = {
                    'url': url,
                    'title': title,
                    'content': content,
                    'status': 'success',
                    'content_length': len(content),
                    'processed_at': time.time(),
                    'attempt': attempt + 1
                }

                results.append(result)
                logger.info(f"Successfully ingested {len(content)} characters from {url}")
                success = True

            except requests.exceptions.Timeout:
                error_message = f'Request timed out after {timeout} seconds (attempt {attempt + 1})'
                logger.warning(f"Timeout error for {url}: {error_message}")
                attempt += 1
                if attempt < max_retries:
                    time.sleep(2 ** attempt)  # Exponential backoff

            except requests.exceptions.ConnectionError as e:
                error_message = f'Connection error: {str(e)} (attempt {attempt + 1})'
                logger.warning(f"Connection error for {url}: {error_message}")
                attempt += 1
                if attempt < max_retries:
                    time.sleep(2 ** attempt)  # Exponential backoff

            except requests.exceptions.HTTPError as e:
                error_message = f'HTTP error: {str(e)} (attempt {attempt + 1})'
                logger.warning(f"HTTP error for {url}: {error_message}")
                # Don't retry on client errors (4xx)
                if 400 <= e.response.status_code < 500:
                    break
                attempt += 1
                if attempt < max_retries:
                    time.sleep(2 ** attempt)  # Exponential backoff

            except requests.exceptions.RequestException as e:
                error_message = f'Request error: {str(e)} (attempt {attempt + 1})'
                logger.warning(f"Request error for {url}: {error_message}")
                attempt += 1
                if attempt < max_retries:
                    time.sleep(2 ** attempt)  # Exponential backoff

            except Exception as e:
                error_message = f'Unexpected error: {str(e)} (attempt {attempt + 1})'
                logger.error(f"Unexpected error for {url}: {error_message}")
                break  # Don't retry on unexpected errors

        # If all attempts failed, add failure result
        if not success:
            error_result = {
                'url': url,
                'status': 'failed',
                'error_message': error_message,
                'processed_at': time.time(),
                'attempts_made': attempt
            }
            results.append(error_result)
            logger.error(f"Failed to ingest {url} after {attempt} attempts: {error_message}")

    return results


def validate_and_clean_url(url: str) -> str:
    """
    Validate and clean a URL.

    Args:
        url: The URL to validate and clean

    Returns:
        The cleaned URL

    Raises:
        ValidationError: If the URL is invalid
    """
    if not validate_url(url):
        raise IngestionError(f"Invalid URL format: {url}")

    # Clean URL by removing fragments
    parsed = urlparse(url)
    clean_url = f"{parsed.scheme}://{parsed.netloc}{parsed.path}"
    if parsed.query:
        clean_url += f"?{parsed.query}"

    return clean_url


def batch_ingest_urls(urls: List[str], delay: float = 1.0, timeout: int = 30) -> List[Dict[str, Any]]:
    """
    Ingest URLs with a delay between requests to be respectful to servers.

    Args:
        urls: List of URLs to ingest
        delay: Delay in seconds between requests
        timeout: Request timeout in seconds

    Returns:
        List of ingestion results
    """
    results = []

    for i, url in enumerate(urls):
        result = ingest_urls([url], timeout)
        results.extend(result)

        # Add delay between requests, except for the last one
        if i < len(urls) - 1:
            time.sleep(delay)

    return results