"""
Environment validation function to verify setup
"""
import os
import sys
from typing import List, Tuple


def validate_environment() -> Tuple[bool, List[str]]:
    """
    Validate the environment setup and return status with any issues found.

    Returns:
        A tuple of (is_valid, list_of_issues)
    """
    issues = []

    # Check Python version
    if sys.version_info < (3, 8):
        issues.append("Python 3.8 or higher is required")

    # Check for .env file
    if not os.path.exists(".env"):
        issues.append(".env file not found in backend directory")

    # Check for required environment variables
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    for var in required_vars:
        if not os.getenv(var):
            issues.append(f"Environment variable {var} is not set")

    # Check for required dependencies
    try:
        import cohere
    except ImportError:
        issues.append("cohere package not installed")

    try:
        import qdrant_client
    except ImportError:
        issues.append("qdrant-client package not installed")

    try:
        import requests
    except ImportError:
        issues.append("requests package not installed")

    try:
        import bs4
    except ImportError:
        issues.append("beautifulsoup4 package not installed")

    try:
        import dotenv
    except ImportError:
        issues.append("python-dotenv package not installed")

    return len(issues) == 0, issues


def main():
    """Main function to run environment validation."""
    print("Validating environment setup...")

    is_valid, issues = validate_environment()

    if is_valid:
        print("✓ Environment validation passed")
        return True
    else:
        print("✗ Environment validation failed:")
        for issue in issues:
            print(f"  - {issue}")
        return False


if __name__ == "__main__":
    main()