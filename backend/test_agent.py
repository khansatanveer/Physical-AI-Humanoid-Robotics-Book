"""
Simple test file to validate the retrieval agent implementation.
"""

from agent import RetrievalAgent


def test_agent_initialization():
    """Test that the agent initializes correctly with required environment variables."""
    try:
        agent = RetrievalAgent()
        print("✓ Agent initialized successfully")
        return True
    except Exception as e:
        print(f"✗ Agent initialization failed: {e}")
        return False


def test_query_processing():
    """Test that the agent can process a simple query."""
    try:
        agent = RetrievalAgent()
        response = agent.query("What is the capital of France?")
        print(f"✓ Query processed successfully: {response['response'][:50]}...")
        return True
    except Exception as e:
        print(f"✗ Query processing failed: {e}")
        return False


def test_no_relevant_documents():
    """Test that the agent handles queries with no relevant documents."""
    try:
        agent = RetrievalAgent()
        response = agent.query("What is the weather like today?")  # This should trigger no-relevance case
        print(f"✓ No-relevance query handled: {response['response'][:50]}...")
        return True
    except Exception as e:
        print(f"✗ No-relevance query handling failed: {e}")
        return False


if __name__ == "__main__":
    print("Running retrieval agent tests...\n")

    tests = [
        test_agent_initialization,
        test_query_processing,
        test_no_relevant_documents
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if test():
            passed += 1
        print()

    print(f"Tests passed: {passed}/{total}")

    if passed == total:
        print("🎉 All tests passed!")
    else:
        print("❌ Some tests failed.")