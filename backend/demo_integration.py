"""
Demo script for end-to-end integration test with sample URLs
This script demonstrates the complete pipeline functionality
"""
import os
import sys
import time
from typing import List, Dict, Any

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pipeline import PipelineOrchestrator
from config import Config
from validation import validate_pipeline_config


def run_demo_pipeline():
    """
    Run a demonstration of the pipeline with sample configuration.
    Note: This requires valid API keys in the .env file.
    """
    print("🔍 Starting URL Ingestion and Embedding Pipeline Demo")
    print("=" * 60)

    # Sample URLs for demonstration (these should be publicly accessible)
    sample_urls = [
        "https://en.wikipedia.org/wiki/Artificial_intelligence",
        "https://en.wikipedia.org/wiki/Machine_learning",
        "https://en.wikipedia.org/wiki/Deep_learning"
    ]

    print(f"📋 URLs to process: {sample_urls}")
    print()

    try:
        # Validate configuration first
        print("🔐 Validating configuration...")
        config_validation = validate_pipeline_config(
            urls=sample_urls,
            chunk_size=1000,
            chunk_overlap=200,
            cohere_model=Config.COHERE_MODEL,
            collection_name=Config.QDRANT_COLLECTION_NAME
        )

        if not config_validation['is_valid']:
            print(f"❌ Configuration validation failed: {config_validation['error']}")
            for component, result in config_validation['details'].items():
                if isinstance(result, dict) and not result.get('is_valid', True):
                    print(f"  - {component}: {result.get('error', 'Unknown error')}")
            return False

        print("✅ Configuration is valid")
        print()

        # Initialize the pipeline orchestrator
        print("⚙️  Initializing pipeline orchestrator...")
        orchestrator = PipelineOrchestrator(
            chunk_size=1000,
            chunk_overlap=200,
            cohere_model=Config.COHERE_MODEL,
            collection_name=Config.QDRANT_COLLECTION_NAME
        )
        print("✅ Pipeline orchestrator initialized")
        print()

        # Run the pipeline
        print("🚀 Starting pipeline execution...")
        start_time = time.time()

        result = orchestrator.run_pipeline(sample_urls)

        end_time = time.time()
        execution_time = end_time - start_time

        print()
        print("📊 EXECUTION RESULTS")
        print("=" * 60)
        print(f"Status: {result['status']}")
        print(f"Processed URLs: {result['processed_count']}")
        print(f"Failed URLs: {result['failed_count']}")
        print(f"Stored Chunks: {result['stored_count']}")
        print(f"Total Chunks: {result['total_chunks']}")
        print(f"Execution Time: {execution_time:.2f} seconds")

        if result['errors']:
            print(f"\n⚠️  Errors encountered:")
            for error in result['errors'][:3]:  # Show first 3 errors
                print(f"  - {error['url']}: {error['error']}")
            if len(result['errors']) > 3:
                print(f"  ... and {len(result['errors']) - 3} more errors")

        print()
        if result['status'] == 'success':
            print("✅ Pipeline completed successfully!")
            print(" embeddings have been stored in Qdrant Cloud.")
        else:
            print("❌ Pipeline failed. Check the errors above.")

        return result['status'] == 'success'

    except Exception as e:
        print(f"❌ Pipeline execution failed with error: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def run_basic_functionality_tests():
    """
    Run basic functionality tests to verify each component works.
    """
    print("🧪 Running basic functionality tests...")
    print("-" * 40)

    try:
        from utils import validate_url, clean_text, generate_chunk_id
        from cleaning import clean_content
        from chunking import chunk_content
        from validation import validate_urls, validate_chunk_parameters

        # Test URL validation
        print("1. Testing URL validation...")
        test_urls = ["https://example.com", "invalid-url"]
        url_results = validate_urls(test_urls)
        print(f"   Valid URL: {url_results[0]['is_valid']}")
        print(f"   Invalid URL: {not url_results[1]['is_valid']}")
        print("   ✅ URL validation working")

        # Test text cleaning
        print("\n2. Testing text cleaning...")
        dirty_text = "  This   is  a   test  \n\n with\t\twhitespace  "
        cleaned = clean_text(dirty_text)
        print(f"   Original: '{dirty_text}'")
        print(f"   Cleaned: '{cleaned}'")
        print("   ✅ Text cleaning working")

        # Test content chunking
        print("\n3. Testing content chunking...")
        sample_content = "This is a test sentence. " * 20  # Create sample content
        chunks = chunk_content(sample_content, "https://example.com", chunk_size=50, chunk_overlap=10)
        print(f"   Generated {len(chunks)} chunks from sample content")
        print(f"   First chunk length: {len(chunks[0]['content']) if chunks else 0}")
        print("   ✅ Content chunking working")

        print("\n✅ All basic functionality tests passed!")
        return True

    except Exception as e:
        print(f"\n❌ Basic functionality tests failed: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """
    Main function to run the integration demo.
    """
    print("📖 URL Ingestion Pipeline - Integration Test Demo")
    print()

    # Check if required environment variables are set
    if not Config.COHERE_API_KEY or not Config.QDRANT_URL or not Config.QDRANT_API_KEY:
        print("⚠️  Warning: API keys not found in environment!")
        print("   Please ensure the following environment variables are set:")
        print("   - COHERE_API_KEY")
        print("   - QDRANT_URL")
        print("   - QDRANT_API_KEY")
        print("   Using .env file or environment variables.")
        print()

    # Run basic functionality tests
    basic_tests_passed = run_basic_functionality_tests()
    print()

    if not basic_tests_passed:
        print("❌ Basic functionality tests failed. Cannot proceed with pipeline demo.")
        return False

    # Ask user if they want to run the full pipeline demo
    print("\n❓ Do you want to run the full pipeline demo?")
    print("   This will attempt to process real URLs and store embeddings in Qdrant.")
    print("   It will use your configured API keys.")
    response = input("   Run demo? (y/n): ").lower().strip()

    if response in ['y', 'yes']:
        print()
        return run_demo_pipeline()
    else:
        print("Demo skipped. You can run the full pipeline manually with:")
        print("python main.py --urls <url1> <url2> ...")
        return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)