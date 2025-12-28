#!/usr/bin/env python3
"""
Performance test script to validate that 95% of queries return within 1 second.
"""

import time
import statistics
from typing import List, Tuple
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from config import Config
from utils.embeddings import EmbeddingsGenerator
from utils.storage import StorageManager
from retrieve import retrieve_content


def run_performance_test(num_queries: int = 20) -> Tuple[List[float], float]:
    """
    Run performance tests and return response times and success rate.

    Args:
        num_queries: Number of queries to run for testing

    Returns:
        Tuple of (response_times, success_rate)
    """
    # Check configuration
    is_valid, msg = Config.validate()
    if not is_valid:
        print(f"Configuration validation failed: {msg}")
        return [], 0.0

    # Initialize components
    try:
        embeddings_gen = EmbeddingsGenerator()
        storage_manager = StorageManager()
        print("Components initialized successfully")
    except Exception as e:
        print(f"Component initialization failed: {str(e)}")
        return [], 0.0

    # Test queries
    test_queries = [
        "What is physical AI?",
        "humanoid robot control",
        "neural networks robotics",
        "machine learning applications",
        "AI in robotics",
        "robotics sensors",
        "control systems",
        "kinematics",
        "ROS 2 architecture",
        "simulation environments"
    ]

    response_times = []
    successful_queries = 0

    print(f"Running {num_queries} queries for performance testing...")

    for i in range(num_queries):
        query = test_queries[i % len(test_queries)]  # Cycle through test queries

        try:
            start_time = time.time()

            results = retrieve_content(
                query=query,
                top_k=5,
                embeddings_gen=embeddings_gen,
                storage_manager=storage_manager
            )

            end_time = time.time()
            response_time = end_time - start_time

            response_times.append(response_time)
            successful_queries += 1

            print(f"Query {i+1}: {response_time:.3f}s - {len(results)} results")

        except Exception as e:
            print(f"Query {i+1} failed: {str(e)}")
            response_times.append(float('inf'))  # Mark failed queries with infinite time

    success_rate = successful_queries / num_queries if num_queries > 0 else 0.0
    return response_times, success_rate


def analyze_performance(response_times: List[float]) -> dict:
    """
    Analyze performance metrics from response times.

    Args:
        response_times: List of response times in seconds

    Returns:
        Dictionary with performance metrics
    """
    if not response_times:
        return {
            "total_queries": 0,
            "successful_queries": 0,
            "failed_queries": 0,
            "avg_response_time": 0,
            "min_response_time": 0,
            "max_response_time": 0,
            "p95_response_time": 0,
            "under_1s_percentage": 0,
            "under_1s_count": 0,
            "threshold_met": False
        }

    # Filter out failed queries (marked with inf)
    successful_times = [t for t in response_times if t != float('inf')]
    failed_count = len(response_times) - len(successful_times)

    if successful_times:
        avg_time = statistics.mean(successful_times)
        min_time = min(successful_times)
        max_time = max(successful_times)

        # Calculate 95th percentile
        sorted_times = sorted(successful_times)
        p95_idx = int(0.95 * len(sorted_times))
        if p95_idx >= len(sorted_times):
            p95_idx = len(sorted_times) - 1
        p95_time = sorted_times[p95_idx]

        # Calculate percentage under 1 second
        under_1s_count = sum(1 for t in successful_times if t < 1.0)
        under_1s_percentage = (under_1s_count / len(successful_times)) * 100

        threshold_met = under_1s_percentage >= 95
    else:
        avg_time = 0
        min_time = 0
        max_time = 0
        p95_time = 0
        under_1s_percentage = 0
        under_1s_count = 0
        threshold_met = False

    return {
        "total_queries": len(response_times),
        "successful_queries": len(successful_times),
        "failed_queries": failed_count,
        "avg_response_time": avg_time,
        "min_response_time": min_time,
        "max_response_time": max_time,
        "p95_response_time": p95_time,
        "under_1s_percentage": under_1s_percentage,
        "under_1s_count": under_1s_count,
        "threshold_met": threshold_met
    }


def main():
    """Main function to run performance tests."""
    print("Running Performance Tests for Retrieval Pipeline")
    print("=" * 50)

    # Run performance tests
    response_times, success_rate = run_performance_test(num_queries=20)

    # Analyze results
    metrics = analyze_performance(response_times)

    # Print results
    print("\nPerformance Test Results:")
    print("-" * 25)
    print(f"Total Queries: {metrics['total_queries']}")
    print(f"Successful Queries: {metrics['successful_queries']}")
    print(f"Failed Queries: {metrics['failed_queries']}")
    print(f"Success Rate: {success_rate:.1%}")

    if metrics['successful_queries'] > 0:
        print(f"\nResponse Time Metrics:")
        print(f"  Average: {metrics['avg_response_time']:.3f}s")
        print(f"  Minimum: {metrics['min_response_time']:.3f}s")
        print(f"  Maximum: {metrics['max_response_time']:.3f}s")
        print(f"  95th Percentile: {metrics['p95_response_time']:.3f}s")

        print(f"\nThreshold Analysis:")
        print(f"  Queries under 1s: {metrics['under_1s_count']}/{metrics['successful_queries']} ({metrics['under_1s_percentage']:.1f}%)")

        if metrics['threshold_met']:
            print(f"  OK 95% of queries under 1 second threshold: MET")
        else:
            print(f"  X 95% of queries under 1 second threshold: NOT MET")

    # Overall result
    print(f"\nOverall Result:")
    if metrics['threshold_met'] and success_rate >= 0.95:
        print("✅ Performance requirements are satisfied")
        return 0
    else:
        print("❌ Performance requirements are not satisfied")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)