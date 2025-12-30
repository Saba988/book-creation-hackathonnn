# Quick Start: URL Ingestion Pipeline

## Setup
1. Ensure Python 3.8+ is installed
2. Install `uv`: `pip install uv`
3. Create project: `mkdir backend && cd backend`
4. Initialize environment: `uv venv`
5. Activate environment: `source .venv/bin/activate` (Linux/Mac) or `source .venv/Scripts/activate` (Windows)

## Configuration
1. Set environment variables:
   - `COHERE_API_KEY`: Your Cohere API key
   - `QDRANT_URL`: Your Qdrant Cloud URL
   - `QDRANT_API_KEY`: Your Qdrant Cloud API key

## Run Pipeline
1. Install dependencies: `uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv`
2. Run: `python main.py --urls "https://example.com/page1,https://example.com/page2"`

## Expected Output
- Processed content chunks stored in Qdrant Cloud
- Processing logs showing progress
- Final summary of processed content

## Configuration Options
- `--chunk-size`: Size of text chunks (default: 1000 characters)
- `--chunk-overlap`: Overlap between chunks (default: 200 characters)
- `--model`: Cohere embedding model to use (default: embed-multilingual-v3.0)
- `--collection`: Qdrant collection name (default: rag_content)

## Troubleshooting
- If you encounter network errors, check your internet connection and URL accessibility
- If embedding generation fails, verify your Cohere API key is valid and within rate limits
- If Qdrant storage fails, check your Qdrant Cloud credentials and storage limits