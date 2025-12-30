"""
Unit tests for the pipeline components
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from pipeline import PipelineOrchestrator
from validation import validate_urls, validate_chunk_parameters, validate_cohere_model
from utils import validate_url, clean_text, generate_chunk_id
from chunking import chunk_content
from entities import ContentChunk, PipelineState


class TestURLValidation(unittest.TestCase):
    """Test URL validation functions."""

    def test_valid_urls(self):
        """Test validation of valid URLs."""
        valid_urls = ["https://example.com", "http://test.org/page"]
        results = validate_urls(valid_urls)

        for result in results:
            self.assertTrue(result['is_valid'])
            self.assertIsNone(result['error'])

    def test_invalid_urls(self):
        """Test validation of invalid URLs."""
        invalid_urls = ["not-a-url", "", "ftp://example.com"]
        results = validate_urls(invalid_urls)

        # First two should be invalid
        self.assertFalse(results[0]['is_valid'])  # not-a-url
        self.assertFalse(results[1]['is_valid'])  # empty string
        self.assertFalse(results[2]['is_valid'])  # ftp scheme


class TestUtils(unittest.TestCase):
    """Test utility functions."""

    def test_validate_url(self):
        """Test URL validation utility."""
        self.assertTrue(validate_url("https://example.com"))
        self.assertTrue(validate_url("http://test.org"))
        self.assertFalse(validate_url("not-a-url"))
        self.assertFalse(validate_url(""))

    def test_clean_text(self):
        """Test text cleaning utility."""
        dirty_text = "  This   is  a   test  \n\n with\t\twhitespace  "
        cleaned = clean_text(dirty_text)
        self.assertEqual(cleaned, "This is a test with whitespace")

    def test_generate_chunk_id(self):
        """Test chunk ID generation."""
        chunk_id = generate_chunk_id("https://example.com", 5)
        self.assertTrue(chunk_id.startswith("chunk_"))
        self.assertIn("_0005", chunk_id)


class TestChunking(unittest.TestCase):
    """Test content chunking functions."""

    def test_chunk_content(self):
        """Test content chunking."""
        content = "This is a test sentence. " * 50  # Create longer content
        url = "https://example.com"

        chunks = chunk_content(content, url, chunk_size=100, chunk_overlap=20)

        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIn('id', chunk)
            self.assertIn('url', chunk)
            self.assertIn('content', chunk)
            self.assertLessEqual(len(chunk['content']), 100)

    def test_chunk_with_small_content(self):
        """Test chunking with content smaller than chunk size."""
        content = "Short content."
        url = "https://example.com"

        chunks = chunk_content(content, url, chunk_size=1000, chunk_overlap=20)

        self.assertEqual(len(chunks), 1)
        self.assertEqual(chunks[0]['content'], content)


class TestValidation(unittest.TestCase):
    """Test validation functions."""

    def test_chunk_parameters_validation(self):
        """Test chunk parameter validation."""
        # Valid parameters
        result = validate_chunk_parameters(1000, 200)
        self.assertTrue(result['is_valid'])
        self.assertIsNone(result['error'])

        # Invalid chunk size
        result = validate_chunk_parameters(0, 200)
        self.assertFalse(result['is_valid'])
        self.assertIsNotNone(result['error'])

        # Invalid overlap
        result = validate_chunk_parameters(1000, 1001)
        self.assertFalse(result['is_valid'])
        self.assertIsNotNone(result['error'])

    def test_cohere_model_validation(self):
        """Test Cohere model validation."""
        # Valid model names
        valid_models = ["embed-multilingual-v3.0", "embed-english-v3.0", "command-r-plus"]

        for model in valid_models:
            result = validate_cohere_model(model)
            self.assertTrue(result['is_valid'], f"Model {model} should be valid")

        # Invalid model name
        result = validate_cohere_model("invalid-model")
        self.assertFalse(result['is_valid'])


class TestEntities(unittest.TestCase):
    """Test entity classes."""

    def test_content_chunk_entity(self):
        """Test ContentChunk entity."""
        chunk = ContentChunk(
            id="test-id",
            url="https://example.com",
            content="Test content",
            section="Test Section"
        )

        self.assertEqual(chunk.id, "test-id")
        self.assertEqual(chunk.url, "https://example.com")
        self.assertEqual(chunk.content, "Test content")
        self.assertEqual(chunk.section, "Test Section")

        # Test to_dict and from_dict
        chunk_dict = chunk.to_dict()
        self.assertIsInstance(chunk_dict, dict)
        self.assertEqual(chunk_dict['id'], "test-id")

        restored_chunk = ContentChunk.from_dict(chunk_dict)
        self.assertEqual(restored_chunk.id, chunk.id)

    def test_pipeline_state_entity(self):
        """Test PipelineState entity."""
        state = PipelineState(
            status="completed",
            processed_urls=["https://example.com"],
            failed_urls=[{"url": "https://error.com", "error": "Not found"}],
            total_chunks=10,
            progress=100.0
        )

        self.assertEqual(state.status, "completed")
        self.assertEqual(len(state.processed_urls), 1)
        self.assertEqual(len(state.failed_urls), 1)
        self.assertEqual(state.total_chunks, 10)
        self.assertEqual(state.progress, 100.0)

        # Test to_dict and from_dict
        state_dict = state.to_dict()
        self.assertIsInstance(state_dict, dict)
        self.assertEqual(state_dict['status'], "completed")

        restored_state = PipelineState.from_dict(state_dict)
        self.assertEqual(restored_state.status, state.status)


class TestPipelineOrchestrator(unittest.TestCase):
    """Test PipelineOrchestrator class."""

    @patch('pipeline.ingest_urls')
    @patch('pipeline.clean_content')
    @patch('pipeline.chunk_content')
    @patch('pipeline.generate_embeddings_for_chunks')
    @patch('pipeline.store_embeddings_in_qdrant')
    def test_run_pipeline_success(self, mock_store, mock_embed, mock_chunk, mock_clean, mock_ingest):
        """Test successful pipeline run."""
        # Mock return values
        mock_ingest.return_value = [
            {'url': 'https://example.com', 'status': 'success', 'content': 'test content', 'title': 'Test'}
        ]
        mock_clean.return_value = 'cleaned test content'
        mock_chunk.return_value = [
            {'id': 'chunk_1', 'url': 'https://example.com', 'content': 'cleaned test content', 'metadata': {}}
        ]
        mock_embed.return_value = [
            {'id': 'chunk_1', 'url': 'https://example.com', 'content': 'cleaned test content', 'embedding': [0.1, 0.2, 0.3], 'metadata': {}}
        ]
        mock_store.return_value = 1

        orchestrator = PipelineOrchestrator(chunk_size=1000, chunk_overlap=200)
        result = orchestrator.run_pipeline(['https://example.com'])

        self.assertEqual(result['status'], 'success')
        self.assertEqual(result['stored_count'], 1)

    def test_pipeline_validation(self):
        """Test pipeline validation."""
        with self.assertRaises(ValueError):
            orchestrator = PipelineOrchestrator(chunk_size=1000, chunk_overlap=200)
            orchestrator.run_pipeline([])  # Empty URLs list should fail validation


if __name__ == '__main__':
    unittest.main()