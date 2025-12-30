"""
Project initialization script
"""
import os
import sys
from pathlib import Path


def init_project():
    """
    Initialize the project structure and verify setup.
    """
    print("Initializing URL Ingestion Pipeline Project...")

    # Create necessary directories
    directories = [
        "backend",
        "backend/data",  # For temporary data storage
        "backend/logs",  # For log files
    ]

    for directory in directories:
        Path(directory).mkdir(exist_ok=True)
        print(f"✓ Created directory: {directory}")

    # Verify required files exist
    required_files = [
        "requirements.txt",
        ".env",
        "main.py",
        "config.py",
        "logging_config.py",
        "errors.py",
        "utils.py"
    ]

    backend_path = Path("backend")
    missing_files = []

    for file in required_files:
        file_path = backend_path / file
        if not file_path.exists():
            missing_files.append(str(file_path))

    if missing_files:
        print(f"✗ Missing required files: {', '.join(missing_files)}")
        return False

    print("✓ All required files are present")

    # Check if we can import required dependencies
    try:
        import cohere
        import qdrant_client
        import requests
        import bs4
        import dotenv
        print("✓ Required dependencies are available")
    except ImportError as e:
        print(f"✗ Missing dependency: {e}")
        return False

    print("Project initialization completed successfully!")
    return True


def main():
    """Main function to run the initialization."""
    success = init_project()
    if not success:
        print("Project initialization failed. Please check the above errors.")
        sys.exit(1)


if __name__ == "__main__":
    main()