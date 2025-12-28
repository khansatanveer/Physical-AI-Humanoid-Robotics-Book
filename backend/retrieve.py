#!/usr/bin/env python3
"""
Book Content Retrieval Pipeline

This module provides functionality to query embedded book content in Qdrant using semantic
similarity search. It accepts user text queries, generates embeddings using Cohere, performs
similarity search against Qdrant, and returns top-k relevant content chunks with metadata.

The pipeline supports:
- Basic semantic search with configurable top-k results
- Validation against known queries with accuracy metrics
- Performance testing across multiple query types
- Consistency testing for repeated queries
- Error handling and service availability checks

Usage:
    python retrieve.py --query "your query text here" --top-k 5
    python retrieve.py --query "test query" --validate
    python retrieve.py --query "test" --performance-test
    python retrieve.py --query "What is physical AI?" --consistency-test

Example:
    # Basic query with default top-5 results
    python retrieve.py --query "What is physical AI?"

    # Query with custom number of results
    python retrieve.py --query "humanoid robot control systems" --top-k 10

    # Query with validation
    python retrieve.py --query "neural networks in robotics" --top-k 3 --validate

Module Functions:
    main() - Main entry point for the retrieval pipeline
    retrieve_content() - Core function to retrieve content chunks based on query
"""

import logging
import argparse
import sys
import time
from typing import List, Dict, Any, Optional

# Import existing utilities
from config import Config
from utils.embeddings import EmbeddingsGenerator
from utils.storage import StorageManager
from utils.retrieval import RetrievedChunk


# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """
    Main entry point for the retrieval pipeline.

    This function handles command-line arguments, initializes the retrieval components,
    and executes the appropriate workflow based on the provided arguments.

    Command-line arguments:
        --query (str): Text query to search for (required)
        --top-k (int): Number of top results to return (default: 5)
        --validate (bool): Run validation against known queries (optional)
        --performance-test (bool): Run performance tests (optional)
        --consistency-test (bool): Run consistency tests for repeated queries (optional)

    The function will:
    1. Parse command-line arguments
    2. Validate configuration
    3. Initialize embeddings generator and storage manager
    4. Execute the appropriate workflow (basic retrieval, validation, performance, or consistency)
    5. Handle errors and provide appropriate logging
    6. Exit with appropriate status code
    """
    parser = argparse.ArgumentParser(description='Book Content Retrieval Pipeline')
    parser.add_argument('--query', type=str, required=True, help='Text query to search for')
    parser.add_argument('--top-k', type=int, default=5, help='Number of top results to return (default: 5)')
    parser.add_argument('--validate', action='store_true', help='Run validation against known queries')
    parser.add_argument('--performance-test', action='store_true', help='Run performance tests')
    parser.add_argument('--consistency-test', action='store_true', help='Run consistency tests for repeated queries')

    args = parser.parse_args()

    logger.info("Starting book content retrieval pipeline...")

    # Validate configuration
    is_valid, msg = Config.validate()
    if not is_valid:
        logger.error(f"Configuration validation failed: {msg}")
        sys.exit(1)

    # Initialize components
    try:
        logger.info("Initializing retrieval components...")
        embeddings_gen = EmbeddingsGenerator()
        storage_manager = StorageManager()
    except Exception as e:
        logger.error(f"Failed to initialize retrieval components: {str(e)}")
        sys.exit(1)

    # Perform retrieval
    try:
        if args.performance_test:
            # Run performance tests
            from utils.retrieval import run_performance_tests, generate_performance_report
            logger.info("Running performance tests...")

            # Define test queries for performance testing
            test_queries = [
                "What is physical AI?",
                "humanoid robot control",
                "neural networks robotics",
                "machine learning applications",
                "AI in robotics"
            ]

            # Run performance tests
            performance_results = run_performance_tests(
                retrieve_func=retrieve_content,
                test_queries=test_queries,
                top_k=args.top_k,
                embeddings_gen=embeddings_gen,
                storage_manager=storage_manager
            )

            # Generate and print performance report
            performance_report = generate_performance_report(performance_results)
            print(f"\n{performance_report}")

        elif args.consistency_test:
            # Run consistency tests
            from utils.retrieval import check_consistency_for_repeated_queries, generate_consistency_report
            logger.info("Running consistency tests...")

            # Run consistency tests for the given query
            consistency_results = check_consistency_for_repeated_queries(
                retrieve_func=retrieve_content,
                query=args.query,
                top_k=args.top_k,
                num_runs=5,  # Run the query 5 times to check consistency
                embeddings_gen=embeddings_gen,
                storage_manager=storage_manager
            )

            # Generate and print consistency report
            consistency_report = generate_consistency_report(consistency_results)
            print(f"\n{consistency_report}")

        elif args.validate:
            # Run validation tests
            from utils.retrieval import create_validation_dataset, run_validation_test
            logger.info("Running validation tests...")

            # Create validation dataset
            validation_dataset = create_validation_dataset()

            # If the query is in the validation dataset, run specific test
            validation_test = None
            for item in validation_dataset:
                if args.query.lower() in item['query'].lower() or item['query'].lower() in args.query.lower():
                    validation_test = item
                    break

            if validation_test:
                logger.info(f"Running validation test for query: {validation_test['query']}")

                # Run the validation test
                test_result = run_validation_test(
                    validation_test['query'],  # query parameter
                    validation_test['expected'],  # expected parameter
                    retrieve_content,  # retrieve_func parameter
                    validation_test['query'],  # first arg for retrieve_content
                    args.top_k,  # second arg for retrieve_content
                    embeddings_gen,  # third arg for retrieve_content
                    storage_manager  # fourth arg for retrieve_content
                )

                # Print validation results
                print(f"\nValidation Test Results:")
                print(f"Query: {test_result['query']}")
                print(f"Expected: {test_result['expected']}")
                print(f"Retrieved Count: {test_result['retrieved_count']}")
                print(f"Accuracy Score: {test_result['accuracy_score']:.3f}")
                print(f"Passed: {'Yes' if test_result['passed'] else 'No'}")

                # Print detailed results if available
                if 'results' in test_result and test_result['results']:
                    print(f"\nTop 3 Retrieved Results:")
                    top_results = test_result['results'][:3]
                    for i, result in enumerate(top_results, 1):
                        if hasattr(result, 'score'):
                            print(f"  {i}. Score: {result.score:.3f}")
                            print(f"     Content: {result.content[:100]}...")
                        else:
                            print(f"  {i}. Score: {result['score']:.3f}")
                            print(f"     Content: {result['payload'].get('content', '')[:100]}...")

                if 'error' in test_result:
                    print(f"Error: {test_result['error']}")
            else:
                # Run standard retrieval
                results = retrieve_content(
                    query=args.query,
                    top_k=args.top_k,
                    embeddings_gen=embeddings_gen,
                    storage_manager=storage_manager
                )

                # Print results
                print(f"\nQuery: {args.query}")
                print(f"Top {len(results)} most similar chunks:")
                for i, result in enumerate(results, 1):
                    if hasattr(result, 'score'):  # If it's a formatted RetrievedChunk object
                        print(f"\n{i}. Score: {result.score:.3f}")
                        print(f"   URL: {result.url}")
                        print(f"   Title: {result.title}")
                        print(f"   Headings: {result.headings}")
                        print(f"   Content: {result.content[:200]}...")
                    else:  # If it's still a raw result from Qdrant
                        print(f"\n{i}. Score: {result['score']:.3f}")
                        print(f"   URL: {result['payload'].get('url', 'N/A')}")
                        print(f"   Title: {result['payload'].get('title', 'N/A')}")
                        print(f"   Headings: {result['payload'].get('headings', [])}")
                        print(f"   Content: {result['payload'].get('content', '')[:200]}...")
        else:
            # Run standard retrieval
            results = retrieve_content(
                query=args.query,
                top_k=args.top_k,
                embeddings_gen=embeddings_gen,
                storage_manager=storage_manager
            )

            # Print results
            print(f"\nQuery: {args.query}")
            print(f"Top {len(results)} most similar chunks:")
            for i, result in enumerate(results, 1):
                if hasattr(result, 'score'):  # If it's a formatted RetrievedChunk object
                    print(f"\n{i}. Score: {result.score:.3f}")
                    print(f"   URL: {result.url}")
                    print(f"   Title: {result.title}")
                    print(f"   Headings: {result.headings}")
                    print(f"   Content: {result.content[:200]}...")
                else:  # If it's still a raw result from Qdrant
                    print(f"\n{i}. Score: {result['score']:.3f}")
                    print(f"   URL: {result['payload'].get('url', 'N/A')}")
                    print(f"   Title: {result['payload'].get('title', 'N/A')}")
                    print(f"   Headings: {result['payload'].get('headings', [])}")
                    print(f"   Content: {result['payload'].get('content', '')[:200]}...")

    except Exception as e:
        error_msg = str(e).lower()
        if any(service_term in error_msg for service_term in ['connection', 'timeout', 'service', 'unavailable', 'qdrant', 'cohere']):
            logger.error(f"Service unavailable: {str(e)}")
            print(f"\nError: Service temporarily unavailable. Please check your connection and configuration.")
        else:
            logger.error(f"Retrieval failed with error: {str(e)}")
            print(f"\nError: {str(e)}")
        sys.exit(1)


def retrieve_content(
    query: str,
    top_k: int,
    embeddings_gen: EmbeddingsGenerator,
    storage_manager: StorageManager
) -> List[RetrievedChunk]:
    """
    Retrieve content chunks relevant to the query using semantic similarity search.

    Args:
        query: The text query to search for
        top_k: Number of top results to return
        embeddings_gen: Embeddings generator instance
        storage_manager: Storage manager instance for Qdrant operations

    Returns:
        List of retrieved chunks with similarity scores and metadata

    Raises:
        Exception: If retrieval fails due to configuration or service issues
    """
    logger.info(f"Starting retrieval for query: '{query[:50]}...' with top_k={top_k}")

    # Validate inputs using retrieval utilities
    from utils.retrieval import validate_query_text, validate_top_k_value

    is_valid, msg = validate_query_text(query)
    if not is_valid:
        raise ValueError(f"Query validation failed: {msg}")

    is_valid, msg = validate_top_k_value(top_k)
    if not is_valid:
        raise ValueError(f"top_k validation failed: {msg}")

    # Collect performance metrics
    metrics = {
        'start_time': time.time(),
        'query_length': len(query),
        'embedding_generation_time': 0,
        'search_time': 0,
        'total_time': 0,
        'results_count': 0,
        'top_k_requested': top_k
    }

    try:
        # Generate embedding for the query
        embedding_start_time = time.time()
        query_embedding = embeddings_gen.generate_embedding_for_chunk({'content': query})
        metrics['embedding_generation_time'] = time.time() - embedding_start_time
        logger.info(f"Generated query embedding in {metrics['embedding_generation_time']:.2f}s")

        # Perform similarity search in Qdrant
        search_start_time = time.time()
        results = storage_manager.search_similar(query_embedding, limit=top_k)
        metrics['search_time'] = time.time() - search_start_time
        logger.info(f"Completed similarity search in {metrics['search_time']:.2f}s")

        # Format retrieved chunks with metadata
        from utils.retrieval import format_retrieved_chunks
        formatted_results = format_retrieved_chunks(results)
        metrics['results_count'] = len(formatted_results)

        # Calculate total time
        metrics['total_time'] = time.time() - metrics['start_time']

        # Log if no results were found
        if not formatted_results:
            logger.warning(f"No relevant results found for query: '{query[:50]}...'")
        else:
            logger.info(f"Retrieved and formatted {len(formatted_results)} results in {metrics['total_time']:.2f}s")

        # Log performance metrics
        logger.info(f"Performance Metrics: "
                   f"Total Time={metrics['total_time']:.2f}s, "
                   f"Embedding Time={metrics['embedding_generation_time']:.2f}s, "
                   f"Search Time={metrics['search_time']:.2f}s, "
                   f"Results Count={metrics['results_count']}")

        return formatted_results
    except Exception as e:
        # Check if this is a Qdrant service availability issue
        error_msg = str(e).lower()
        if any(service_term in error_msg for service_term in ['connection', 'timeout', 'service', 'unavailable', 'qdrant']):
            logger.error(f"Qdrant service unavailable: {str(e)}")
        else:
            logger.error(f"Error during retrieval: {str(e)}")
        raise


if __name__ == "__main__":
    main()