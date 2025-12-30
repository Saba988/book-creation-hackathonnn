# URL Ingestion and Embedding Pipeline for RAG System

This project implements a complete pipeline for RAG (Retrieval Augmented Generation) that handles URL ingestion, content processing, embedding generation, and vector storage. The pipeline extracts content from specified URLs, processes it, generates embeddings using Cohere models, and stores the vectors in Qdrant Cloud for semantic search capabilities.

## Table of Contents
- [Features](#features)
- [Setup](#setup)
- [Configuration](#configuration)
- [Installation](#installation)
- [Usage](#usage)
- [Command Line Options](#command-line-options)
- [Examples](#examples)
- [Project Structure](#project-structure)
- [Error Handling](#error-handling)
- [Troubleshooting](#troubleshooting)
- [Performance Considerations](#performance-considerations)

## Features

- **URL Ingestion**: Extract content from multiple URLs with robust error handling
- **Content Processing**: Clean and structure content with metadata preservation
- **Smart Chunking**: Configurable content chunking with overlap support
- **Embedding Generation**: Generate embeddings using Cohere's latest models
- **Vector Storage**: Store embeddings with rich metadata in Qdrant Cloud
- **Progress Tracking**: Real-time progress monitoring with detailed metrics
- **Error Recovery**: Automatic retry mechanisms for network issues
- **Comprehensive Logging**: Detailed logging for operational visibility

## Setup

1. Ensure Python 3.8+ is installed
2. Install `uv`: `pip install uv`
3. Navigate to the backend directory: `cd backend`
4. Initialize environment: `uv venv`
5. Activate environment: `source .venv/bin/activate` (Linux/Mac) or `source .venv/Scripts/activate` (Windows)

## Configuration

1. Copy the `.env` template: `cp .env .env.local`
2. Set environment variables in `.env.local`:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant Cloud URL
   - `QDRANT_API_KEY`: Your Qdrant Cloud API key

### Configuration Parameters

- `CHUNK_SIZE`: Size of text chunks (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 200)
- `COHERE_MODEL`: Cohere embedding model to use (default: embed-multilingual-v3.0)
- `QDRANT_COLLECTION_NAME`: Qdrant collection name (default: rag_content)

## Installation

1. Install dependencies: `uv pip install -r requirements.txt`
2. Or install directly: `uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv`

## Usage

Run the pipeline with:
```bash
python main.py --urls "https://example.com/page1" "https://example.com/page2" --chunk-size 1000 --chunk-overlap 200
```

## Command Line Options

- `--urls`: List of URLs to process (required)
- `--chunk-size`: Size of text chunks (default: 1000)
- `--chunk-overlap`: Overlap between chunks (default: 200)
- `--model`: Cohere embedding model to use (default: embed-multilingual-v3.0)
- `--collection`: Qdrant collection name (default: rag_content)
- `--log-level`: Logging level (DEBUG, INFO, WARNING, ERROR)

## Examples

### Basic Usage
```bash
python main.py --urls "https://example.com/docs/intro" "https://example.com/docs/setup" --log-level INFO
```

### Advanced Configuration
```bash
python main.py --urls "https://example.com/page1" "https://example.com/page2" \
  --chunk-size 500 \
  --chunk-overlap 100 \
  --model embed-multilingual-v3.0 \
  --collection my_documents \
  --log-level DEBUG
```

### Processing Multiple URLs
```bash
python main.py --urls "https://site1.com/page" "https://site2.com/page" "https://site3.com/page" \
  --chunk-size 800 \
  --log-level INFO
```

## Project Structure

```
backend/
├── main.py                 # Main entry point that orchestrates the complete pipeline
├── config.py              # Configuration handling for API keys and settings
├── logging_config.py      # Logging setup and configuration
├── errors.py              # Custom exception classes
├── utils.py               # Utility functions for string handling and validation
├── validation.py          # Input validation for URLs and configuration parameters
├── metrics.py             # Pipeline progress reporting and metrics
├── ingestion.py           # URL ingestion and content extraction
├── cleaning.py            # Content cleaning module
├── chunking.py            # Content chunking module
├── embedding.py           # Embedding generation module
├── storage.py             # Qdrant storage module
├── qdrant_schema.py       # Qdrant collection schema definitions
├── entities.py            # Data entity definitions
├── pipeline.py            # Pipeline orchestrator class
├── requirements.txt       # Python dependencies
├── .env                   # Environment variables template
├── README.md              # This file
└── pipeline.log           # Log file (generated at runtime)
```

## Error Handling

The pipeline implements comprehensive error handling:

- **Network Errors**: Automatic retries with exponential backoff
- **Content Processing**: Graceful handling of malformed content
- **API Limits**: Rate limiting and retry mechanisms
- **Storage Issues**: Connection resilience and retry logic
- **Validation**: Input validation with detailed error messages

## Troubleshooting

For detailed troubleshooting information, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

### Quick Checks

1. **API Key Errors**: Verify your Cohere and Qdrant API keys are correct and active
2. **URL Access Issues**: Ensure URLs are publicly accessible and properly formatted
3. **Rate Limits**: If encountering rate limit errors, reduce the number of concurrent requests
4. **Memory Issues**: For large documents, reduce chunk size or process in smaller batches

### Log Files

Check `pipeline.log` for detailed execution information and error details.

### Debugging

Run with `--log-level DEBUG` to get detailed information about each processing step.

## Performance Considerations

- **Chunk Size**: Larger chunks reduce the number of embeddings but may lose granularity
- **Batch Processing**: The pipeline processes content in batches for optimal performance
- **Network Delays**: URL ingestion includes delays to respect server limits
- **API Costs**: Embedding generation costs scale with the number of chunks generated

## Integration

The pipeline can be integrated into larger RAG systems by using the `PipelineOrchestrator` class directly:

```python
from pipeline import PipelineOrchestrator

orchestrator = PipelineOrchestrator(chunk_size=1000, chunk_overlap=200)
result = orchestrator.run_pipeline(['https://example.com'])
```