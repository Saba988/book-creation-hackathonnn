# Data Models: URL Ingestion Pipeline

## Content Chunk Entity
- **id**: Unique identifier for the chunk (string, required)
- **url**: Source URL of the content (string, required)
- **section**: Section/heading from the source (string, optional)
- **content**: Cleaned text content of the chunk (string, required)
- **metadata**: Additional metadata dictionary (object, optional)
- **embedding**: Vector representation of the content (array of numbers, required)
- **created_at**: Timestamp when chunk was created (ISO string, required)

## Pipeline State Entity
- **status**: Current stage of the pipeline (string, required: "initialized", "ingesting", "cleaning", "chunking", "embedding", "storing", "completed", "failed")
- **processed_urls**: List of URLs successfully processed (array of strings, optional)
- **failed_urls**: List of URLs that failed processing (array of objects with url and error, optional)
- **total_chunks**: Total number of content chunks created (number, optional)
- **progress**: Progress percentage (number, 0-100, optional)
- **timestamp**: When the state was recorded (ISO string, required)

## Embedding Result Entity
- **chunk_id**: ID of the content chunk (string, required)
- **embedding_vector**: The numerical embedding vector (array of numbers, required)
- **text_length**: Length of the original text (number, required)
- **processing_time**: Time taken to generate embedding in milliseconds (number, required)

## URL Processing Result Entity
- **url**: The processed URL (string, required)
- **status**: Processing status (string, required: "success", "failed", "skipped")
- **title**: Page title (string, optional)
- **content_length**: Length of extracted content (number, optional)
- **error_message**: Error details if processing failed (string, optional)
- **processed_at**: When processing was completed (ISO string, required)