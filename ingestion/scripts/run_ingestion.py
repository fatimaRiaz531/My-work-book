#!/usr/bin/env python3
"""
Script to run the ingestion pipeline for the RAG chatbot.
"""
import sys
import os
from pathlib import Path

# Add the parent directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.main import run_ingestion_pipeline


def main():
    """Main entry point for the ingestion script."""
    print("Starting RAG Chatbot Ingestion Pipeline...")
    print("=" * 50)

    try:
        run_ingestion_pipeline()
        print("=" * 50)
        print("Ingestion pipeline completed successfully!")
        return 0
    except KeyboardInterrupt:
        print("\nIngestion pipeline interrupted by user.")
        return 1
    except Exception as e:
        print(f"Error running ingestion pipeline: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())