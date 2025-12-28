"""
Quickstart validation script to ensure all features work as expected.
"""

import os
from agent import RetrievalAgent


def validate_setup():
    """Validate that the basic setup works correctly."""
    print("Validating setup...")

    # Check environment variables
    required_vars = ["OPENROUTER_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"❌ Missing environment variables: {missing_vars}")
        print("Please set these variables in your .env file")
        return False

    print("✓ Environment variables validated")
    return True


def validate_agent_initialization():
    """Validate that the agent can be initialized."""
    print("\nValidating agent initialization...")

    try:
        agent = RetrievalAgent()
        print("✓ Agent initialized successfully")
        return True
    except Exception as e:
        print(f"❌ Agent initialization failed: {e}")
        return False


def validate_query_functionality():
    """Validate that the agent can process queries."""
    print("\nValidating query functionality...")

    try:
        agent = RetrievalAgent()

        # Test 1: A query that should have relevant documents
        response1 = agent.query("What are the key principles of AI?")
        print(f"✓ Query processed: {response1['response'][:60]}...")

        # Test 2: A query that should not have relevant documents
        response2 = agent.query("What is the weather like today?")
        print(f"✓ No-relevance query handled: {response2['response'][:60]}...")

        return True
    except Exception as e:
        print(f"❌ Query functionality failed: {e}")
        return False


def validate_interactive_mode():
    """Validate that the interactive mode is available."""
    print("\nValidating interactive mode...")

    try:
        agent = RetrievalAgent()
        # The interactive mode method exists and is callable
        assert callable(agent.interactive_mode)
        print("✓ Interactive mode available")
        return True
    except Exception as e:
        print(f"❌ Interactive mode validation failed: {e}")
        return False


def main():
    """Run all validation tests."""
    print("Running quickstart validation...\n")

    tests = [
        validate_setup,
        validate_agent_initialization,
        validate_query_functionality,
        validate_interactive_mode
    ]

    passed = 0
    total = len(tests)

    for test in tests:
        if test():
            passed += 1

    print(f"\nValidation Results: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All validation tests passed! The agent is ready to use.")
        print("\nNext steps:")
        print("1. Run 'python agent.py' for interactive mode")
        print("2. Run 'uvicorn api:app --reload' for API mode")
        print("3. See README.md for full usage instructions")
        return True
    else:
        print("❌ Some validation tests failed. Please check the output above.")
        return False


if __name__ == "__main__":
    main()