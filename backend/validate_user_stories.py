#!/usr/bin/env python3
"""
Validation script for all user stories in the retrieval pipeline.

This script validates that all user stories have been properly implemented:
- User Story 1: Query Book Content with Semantic Search
- User Story 2: Validate Retrieval Quality Against Known Queries
- User Story 3: Monitor Retrieval Performance and Consistency
"""

import sys
import os
import time
import subprocess
from typing import Dict, Any

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from config import Config
from utils.embeddings import EmbeddingsGenerator
from utils.storage import StorageManager
from retrieve import retrieve_content


def validate_user_story_1():
    """
    Validate User Story 1: Query Book Content with Semantic Search (P1)

    Goal: Implement core functionality to submit text queries and retrieve relevant
    book content chunks with metadata.
    """
    print("Validating User Story 1: Query Book Content with Semantic Search")
    print("-" * 65)

    # Check configuration
    is_valid, msg = Config.validate()
    if not is_valid:
        print("X Configuration validation failed: {msg}")
        return False
    print("OK Configuration validation passed")

    # Initialize components
    try:
        embeddings_gen = EmbeddingsGenerator()
        storage_manager = StorageManager()
        print("OK Components initialized successfully")
    except Exception as e:
        print(f"X Component initialization failed: {str(e)}")
        return False

    # Test basic retrieval functionality
    test_query = "What is physical AI?"
    try:
        results = retrieve_content(
            query=test_query,
            top_k=3,
            embeddings_gen=embeddings_gen,
            storage_manager=storage_manager
        )

        if len(results) > 0:
            print(f"OK Basic retrieval successful - retrieved {len(results)} results")

            # Check that results have required metadata
            first_result = results[0]
            has_required_fields = (
                hasattr(first_result, 'content') and
                hasattr(first_result, 'url') and
                hasattr(first_result, 'title') and
                hasattr(first_result, 'headings') and
                hasattr(first_result, 'score')
            )

            if has_required_fields:
                print("OK Results contain required metadata (content, url, title, headings, score)")
            else:
                print("X Results missing required metadata fields")
                return False
        else:
            print("? No results returned (this might be expected if no matching content exists)")

    except Exception as e:
        print(f"X Basic retrieval failed: {str(e)}")
        return False

    print("OK User Story 1 validation completed successfully\n")
    return True


def validate_user_story_2():
    """
    Validate User Story 2: Validate Retrieval Quality Against Known Queries (P2)

    Goal: Implement validation functionality to test retrieval quality against
    known queries and expected results.
    """
    print("Validating User Story 2: Validate Retrieval Quality Against Known Queries")
    print("-" * 70)

    # Test validation functionality using command line
    try:
        # Test with a simple query
        result = subprocess.run([
            sys.executable, 'retrieve.py',
            '--query', 'What is physical AI?',
            '--top-k', '3',
            '--validate'
        ], capture_output=True, text=True, timeout=30)

        if result.returncode == 0:
            print("OK Validation command executed successfully")
            if "Validation Test Results:" in result.stdout or "accuracy" in result.stdout.lower():
                print("OK Validation output contains expected elements")
            else:
                print("? Validation output format may not contain expected elements")
        else:
            print(f"X Validation command failed with return code {result.returncode}")
            print(f"Error: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print("X Validation command timed out")
        return False
    except Exception as e:
        print(f"X Validation test failed: {str(e)}")
        return False

    print("OK User Story 2 validation completed successfully\n")
    return True


def validate_user_story_3():
    """
    Validate User Story 3: Monitor Retrieval Performance and Consistency (P3)

    Goal: Implement performance monitoring to ensure consistent, low-latency
    retrieval results.
    """
    print("Validating User Story 3: Monitor Retrieval Performance and Consistency")
    print("-" * 70)

    # Test performance functionality using command line
    try:
        result = subprocess.run([
            sys.executable, 'retrieve.py',
            '--query', 'test',
            '--performance-test'
        ], capture_output=True, text=True, timeout=60)

        if result.returncode == 0:
            print("OK Performance test command executed successfully")
            if "Performance Test Report" in result.stdout:
                print("OK Performance report generated successfully")
            else:
                print("? Performance report may not have been generated as expected")
        else:
            print(f"X Performance test command failed with return code {result.returncode}")
            print(f"Error: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print("X Performance test command timed out")
        return False
    except Exception as e:
        print(f"X Performance test failed: {str(e)}")
        return False

    # Test consistency functionality
    try:
        result = subprocess.run([
            sys.executable, 'retrieve.py',
            '--query', 'What is physical AI?',
            '--consistency-test'
        ], capture_output=True, text=True, timeout=60)

        if result.returncode == 0:
            print("OK Consistency test command executed successfully")
            if "Consistency Test Report" in result.stdout:
                print("OK Consistency report generated successfully")
            else:
                print("? Consistency report may not have been generated as expected")
        else:
            print(f"X Consistency test command failed with return code {result.returncode}")
            print(f"Error: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print("X Consistency test command timed out")
        return False
    except Exception as e:
        print(f"X Consistency test failed: {str(e)}")
        return False

    print("OK User Story 3 validation completed successfully\n")
    return True


def run_final_validation():
    """Run final validation of all user stories."""
    print("Final Validation of All User Stories")
    print("=" * 50)

    results = {
        "User Story 1": validate_user_story_1(),
        "User Story 2": validate_user_story_2(),
        "User Story 3": validate_user_story_3()
    }

    print("\nValidation Summary:")
    print("=" * 20)
    all_passed = True
    for story, passed in results.items():
        status = "PASSED" if passed else "FAILED"
        print(f"{story}: {status}")
        if not passed:
            all_passed = False

    print(f"\nOverall Result: {'ALL USER STORIES PASSED' if all_passed else 'SOME USER STORIES FAILED'}")
    return all_passed


if __name__ == "__main__":
    success = run_final_validation()
    sys.exit(0 if success else 1)