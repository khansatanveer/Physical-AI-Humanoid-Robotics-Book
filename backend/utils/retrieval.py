"""
Utility functions for the retrieval pipeline.
Contains retrieval-specific functionality that supports the main retrieve.py module.
"""

import logging
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
import time
from config import Config


logger = logging.getLogger(__name__)


@dataclass
class Query:
    """Represents a text query for content retrieval."""
    text: str
    embedding: Optional[List[float]] = None
    timestamp: Optional[float] = None
    top_k: int = 5


@dataclass
class RetrievedChunk:
    """Represents a content chunk retrieved from the vector database."""
    id: str
    content: str
    url: str
    title: str
    headings: List[str]
    score: float
    metadata: Dict[str, Any]


@dataclass
class SimilarityScore:
    """Represents a similarity score between query and content."""
    value: float
    algorithm: str = "cosine"


def validate_query_text(query_text: str) -> Tuple[bool, str]:
    """
    Validate the query text input.

    Args:
        query_text: The text to validate

    Returns:
        Tuple of (is_valid, error_message)
    """
    if not query_text or not query_text.strip():
        return False, "Query text cannot be empty"

    if len(query_text.strip()) < 3:
        return False, "Query text must be at least 3 characters long"

    if len(query_text) > 10000:  # Arbitrary limit to prevent extremely long queries
        return False, "Query text is too long (max 10000 characters)"

    # Check for potentially problematic characters or patterns
    try:
        # Ensure the text is valid UTF-8
        query_text.encode('utf-8')
    except UnicodeError:
        return False, "Query contains invalid characters"

    return True, "Query text is valid"


def retry_with_backoff(func, max_retries: int = 3, backoff_factor: float = 1.0, exceptions: tuple = (Exception,)):
    """
    Retry a function with exponential backoff.

    Args:
        func: The function to retry
        max_retries: Maximum number of retry attempts
        backoff_factor: Factor for exponential backoff (1, 2, 4, 8, ...)
        exceptions: Tuple of exceptions that should trigger a retry

    Returns:
        Result of the function call
    """
    import time
    import random

    for attempt in range(max_retries + 1):
        try:
            return func()
        except exceptions as e:
            if attempt == max_retries:
                # Last attempt, raise the exception
                raise e

            # Calculate delay with exponential backoff and jitter
            delay = backoff_factor * (2 ** attempt) + random.uniform(0, 1)
            logger.info(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay:.2f}s...")
            time.sleep(delay)

    # This line should never be reached
    raise Exception("Retry logic error")


def format_retrieved_chunks(results: List[Dict[str, Any]]) -> List[RetrievedChunk]:
    """
    Format raw search results into RetrievedChunk objects.

    Args:
        results: Raw search results from Qdrant

    Returns:
        List of RetrievedChunk objects with properly formatted data
    """
    formatted_chunks = []

    for result in results:
        payload = result.get('payload', {})
        metadata = payload.get('metadata', {})

        chunk = RetrievedChunk(
            id=result.get('id', ''),
            content=payload.get('content', ''),
            url=payload.get('url', ''),
            title=payload.get('title', ''),
            headings=payload.get('headings', []),
            score=result.get('score', 0.0),
            metadata=metadata
        )
        formatted_chunks.append(chunk)

    return formatted_chunks


def calculate_accuracy_score(retrieved_chunks: List[RetrievedChunk], expected_content: str) -> float:
    """
    Calculate accuracy score based on similarity to expected content.

    Args:
        retrieved_chunks: List of retrieved chunks to evaluate
        expected_content: The expected content to compare against

    Returns:
        Accuracy score between 0.0 and 1.0
    """
    if not retrieved_chunks:
        return 0.0

    # Simple similarity calculation based on content overlap
    expected_lower = expected_content.lower()
    total_score = 0.0
    valid_chunks = 0

    for chunk in retrieved_chunks:
        chunk_content = chunk.content.lower()
        # Calculate overlap as a simple heuristic
        common_words = set(expected_lower.split()) & set(chunk_content.split())
        if common_words:
            overlap_score = len(common_words) / len(set(expected_lower.split()))
            total_score += min(overlap_score, chunk.score)  # Use minimum of overlap and similarity score
            valid_chunks += 1

    if valid_chunks == 0:
        return 0.0

    return total_score / valid_chunks


def measure_retrieval_performance(func, *args, **kwargs) -> Tuple[Any, float]:
    """
    Measure the performance of a retrieval function.

    Args:
        func: The function to measure
        *args: Arguments to pass to the function
        **kwargs: Keyword arguments to pass to the function

    Returns:
        Tuple of (function result, execution time in seconds)
    """
    start_time = time.time()
    result = func(*args, **kwargs)
    execution_time = time.time() - start_time

    return result, execution_time


def validate_top_k_value(top_k: int) -> Tuple[bool, str]:
    """
    Validate the top_k parameter value.

    Args:
        top_k: The number of results to retrieve

    Returns:
        Tuple of (is_valid, error_message)
    """
    if top_k < 1:
        return False, "top_k must be a positive integer (at least 1)"

    if top_k > 100:
        return False, "top_k must not exceed 100"

    return True, "top_k value is valid"


def create_validation_dataset() -> List[Dict[str, str]]:
    """
    Create a basic validation dataset with known queries and expected results.

    Returns:
        List of dictionaries with 'query' and 'expected' keys
    """
    # This is a basic dataset - in a real implementation, this would be loaded
    # from a file or database with actual known query-result pairs
    return [
        {
            "query": "What is physical AI?",
            "expected": "Physical AI combines artificial intelligence with physical systems"
        },
        {
            "query": "humanoid robot control",
            "expected": "Humanoid robots require sophisticated control systems"
        },
        {
            "query": "neural networks robotics",
            "expected": "Neural networks enable adaptive behavior in robotics"
        }
    ]


def run_validation_test(query: str, expected: str, retrieve_func, *args, **kwargs) -> Dict[str, Any]:
    """
    Run a validation test for a known query against expected results.

    Args:
        query: The query text to test
        expected: The expected content to match against
        retrieve_func: The retrieval function to call
        *args: Arguments to pass to the retrieval function
        **kwargs: Keyword arguments to pass to the retrieval function

    Returns:
        Dictionary with test results including accuracy score
    """
    try:
        # Run the retrieval
        results = retrieve_func(*args, **kwargs)

        # Calculate accuracy
        accuracy = calculate_accuracy_score(results, expected) if results else 0.0

        # Determine if the test passed (accuracy above threshold)
        passed = accuracy >= 0.85  # Using 85% as the required threshold for validation

        return {
            "query": query,
            "expected": expected,
            "retrieved_count": len(results) if results else 0,
            "accuracy_score": accuracy,
            "passed": passed,
            "results": results
        }
    except Exception as e:
        return {
            "query": query,
            "expected": expected,
            "error": str(e),
            "passed": False,
            "accuracy_score": 0.0
        }


def run_performance_tests(retrieve_func, test_queries: List[str], top_k: int = 5, *args, **kwargs) -> Dict[str, Any]:
    """
    Run performance tests on various query types.

    Args:
        retrieve_func: The retrieval function to test
        test_queries: List of different query types to test
        top_k: Number of results to retrieve for each query
        *args: Additional arguments to pass to the retrieval function
        **kwargs: Additional keyword arguments to pass to the retrieval function

    Returns:
        Dictionary with performance metrics for all queries
    """
    results = {
        "total_queries": len(test_queries),
        "successful_queries": 0,
        "failed_queries": 0,
        "total_time": 0,
        "avg_time": 0,
        "min_time": float('inf'),
        "max_time": 0,
        "throughput": 0,
        "query_performance": [],
        "threshold_met": 0,  # Count of queries that met <1 second threshold
        "threshold_percentage": 0
    }

    successful_times = []

    for i, query in enumerate(test_queries):
        try:
            query_start_time = time.time()
            query_results = retrieve_func(query=query, top_k=top_k, *args, **kwargs)
            query_time = time.time() - query_start_time

            successful_times.append(query_time)

            query_performance = {
                "query": query,
                "time": query_time,
                "results_count": len(query_results),
                "under_threshold": query_time < 1.0  # Check if under 1 second
            }

            results["query_performance"].append(query_performance)
            results["successful_queries"] += 1

            if query_time < 1.0:
                results["threshold_met"] += 1

        except Exception as e:
            results["failed_queries"] += 1
            logger.error(f"Performance test failed for query '{query}': {str(e)}")

    if successful_times:
        results["total_time"] = sum(successful_times)
        results["avg_time"] = results["total_time"] / len(successful_times)
        results["min_time"] = min(successful_times)
        results["max_time"] = max(successful_times)
        results["throughput"] = len(successful_times) / results["total_time"] if results["total_time"] > 0 else 0
        results["threshold_percentage"] = (results["threshold_met"] / len(test_queries)) * 100
    else:
        results["avg_time"] = 0
        results["min_time"] = 0
        results["max_time"] = 0
        results["throughput"] = 0
        results["threshold_percentage"] = 0

    return results


def check_consistency_for_repeated_queries(
    retrieve_func,
    query: str,
    top_k: int,
    num_runs: int = 5,
    *args,
    **kwargs
) -> Dict[str, Any]:
    """
    Check consistency of results across repeated queries.

    Args:
        retrieve_func: The retrieval function to test
        query: The query text to test repeatedly
        top_k: Number of results to retrieve
        num_runs: Number of times to run the query (default: 5)
        *args: Additional arguments to pass to the retrieval function
        **kwargs: Additional keyword arguments to pass to the retrieval function

    Returns:
        Dictionary with consistency metrics
    """
    results = {
        "query": query,
        "num_runs": num_runs,
        "successful_runs": 0,
        "failed_runs": 0,
        "consistency_score": 0.0,
        "avg_response_time": 0.0,
        "response_time_variance": 0.0,
        "results_varied": False,
        "run_details": []
    }

    response_times = []
    all_results = []

    for i in range(num_runs):
        try:
            run_start_time = time.time()
            run_results = retrieve_func(query=query, top_k=top_k, *args, **kwargs)
            run_time = time.time() - run_start_time

            response_times.append(run_time)
            all_results.append(run_results)

            run_detail = {
                "run": i + 1,
                "time": run_time,
                "result_count": len(run_results),
                "first_result_id": run_results[0].id if run_results and hasattr(run_results[0], 'id') else run_results[0]['id'] if run_results else None
            }
            results["run_details"].append(run_detail)
            results["successful_runs"] += 1

        except Exception as e:
            results["failed_runs"] += 1
            logger.error(f"Consistency test failed for run {i + 1}: {str(e)}")

    if response_times:
        # Calculate average response time
        results["avg_response_time"] = sum(response_times) / len(response_times)

        # Calculate response time variance
        if len(response_times) > 1:
            mean_time = results["avg_response_time"]
            variance = sum((t - mean_time) ** 2 for t in response_times) / len(response_times)
            results["response_time_variance"] = variance
        else:
            results["response_time_variance"] = 0

        # Check result consistency (comparing first result IDs across runs)
        if all_results and len(all_results) > 1:
            first_result_ids = []
            for run_results in all_results:
                if run_results:
                    first_id = run_results[0].id if hasattr(run_results[0], 'id') else run_results[0]['id']
                    first_result_ids.append(first_id)

            # Calculate consistency score as percentage of runs with same first result
            if first_result_ids:
                most_common_id = max(set(first_result_ids), key=first_result_ids.count)
                consistent_runs = first_result_ids.count(most_common_id)
                results["consistency_score"] = consistent_runs / len(first_result_ids)
                results["results_varied"] = len(set(first_result_ids)) > 1

    return results


def generate_performance_report(performance_results: Dict[str, Any]) -> str:
    """
    Generate a formatted performance report from performance test results.

    Args:
        performance_results: Results from run_performance_tests

    Returns:
        Formatted performance report as a string
    """
    report = [
        "Performance Test Report",
        "=" * 50,
        f"Total Queries: {performance_results['total_queries']}",
        f"Successful Queries: {performance_results['successful_queries']}",
        f"Failed Queries: {performance_results['failed_queries']}",
        "",
        "Response Time Metrics:",
        f"  Average Time: {performance_results['avg_time']:.3f}s",
        f"  Min Time: {performance_results['min_time']:.3f}s",
        f"  Max Time: {performance_results['max_time']:.3f}s",
        f"  Total Time: {performance_results['total_time']:.3f}s",
        "",
        "Performance Metrics:",
        f"  Throughput: {performance_results['throughput']:.2f} queries/second",
        f"  Threshold Compliance: {performance_results['threshold_percentage']:.1f}% (<1s)",
        f"  Threshold Met: {performance_results['threshold_met']}/{performance_results['total_queries']} queries",
        "",
        "Query Performance Details:"
    ]

    for detail in performance_results['query_performance']:
        status = "OK" if detail['under_threshold'] else "X"
        report.append(f"  [{status}] Query: '{detail['query'][:50]}...'")
        report.append(f"    Time: {detail['time']:.3f}s, Results: {detail['results_count']}")

    return "\n".join(report)


def generate_consistency_report(consistency_results: Dict[str, Any]) -> str:
    """
    Generate a formatted consistency report from consistency test results.

    Args:
        consistency_results: Results from check_consistency_for_repeated_queries

    Returns:
        Formatted consistency report as a string
    """
    report = [
        "Consistency Test Report",
        "=" * 50,
        f"Query: '{consistency_results['query'][:50]}...'",
        f"Number of Runs: {consistency_results['num_runs']}",
        f"Successful Runs: {consistency_results['successful_runs']}",
        f"Failed Runs: {consistency_results['failed_runs']}",
        "",
        "Consistency Metrics:",
        f"  Consistency Score: {consistency_results['consistency_score']:.2f}",
        f"  Average Response Time: {consistency_results['avg_response_time']:.3f}s",
        f"  Response Time Variance: {consistency_results['response_time_variance']:.6f}",
        f"  Results Varied: {'Yes' if consistency_results['results_varied'] else 'No'}",
        "",
        "Run Details:"
    ]

    for detail in consistency_results['run_details']:
        report.append(f"  Run {detail['run']}: {detail['time']:.3f}s, {detail['result_count']} results")

    return "\n".join(report)