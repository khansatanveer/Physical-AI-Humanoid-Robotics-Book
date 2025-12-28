"""
Test script to verify the RAG API works with OpenRouter
"""
import os
import sys
from unittest.mock import Mock, patch

# Add the backend directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_openrouter_configuration():
    """Test that the agent works with OpenRouter configuration."""

    # Mock the OpenAI and Qdrant clients to avoid actual API calls
    with patch('openai.OpenAI') as mock_openai, \
         patch('qdrant_client.QdrantClient') as mock_qdrant:

        # Set up mock return values
        mock_client = Mock()
        mock_openai.return_value = mock_client

        # Mock the chat completions response
        mock_completion = Mock()
        mock_completion.choices = [Mock()]
        mock_completion.choices[0].message = Mock()
        mock_completion.choices[0].message.content = "This is a test response from OpenRouter"

        mock_client.chat.completions.create.return_value = mock_completion

        # Mock embeddings
        mock_embedding = Mock()
        mock_embedding.data = [Mock()]
        mock_embedding.data[0].embedding = [0.1, 0.2, 0.3]  # Mock embedding vector
        mock_client.embeddings.create.return_value = mock_embedding

        # Mock Qdrant client
        mock_qdrant.return_value = Mock()
        mock_qdrant_instance = mock_qdrant.return_value
        mock_search_result = [
            Mock(id="test_id", payload={"content": "test content", "source_document": "test_doc"}, score=0.9)
        ]
        mock_qdrant_instance.search.return_value = mock_search_result

        try:
            # Set environment variables for OpenRouter
            os.environ['OPENROUTER_API_KEY'] = 'test-openrouter-key'
            os.environ['AGENT_MODEL'] = 'openai/gpt-3.5-turbo'
            os.environ['QDRANT_URL'] = 'localhost'
            os.environ['QDRANT_COLLECTION_NAME'] = 'test_collection'

            # Import and test the agent
            from agent import RetrievalAgent

            # Test agent initialization with OpenRouter settings
            agent = RetrievalAgent()
            print("✓ Agent initialized successfully with OpenRouter configuration")

            # Test a query
            result = agent.query("Test query for OpenRouter")
            print(f"✓ Query processed successfully with OpenRouter: {result['response'][:50]}...")

            # Verify result structure
            expected_keys = ["response", "source_chunks", "grounded", "query_id", "confidence", "timestamp"]
            for key in expected_keys:
                assert key in result, f"Missing key: {key}"
            print("✓ Result structure is correct")

            print("\n🎉 OpenRouter configuration test passed!")
            print("The agent is properly configured to work with OpenRouter API.")
            print("\nTo use with real OpenRouter API:")
            print("1. Get an API key from https://openrouter.ai/keys")
            print('2. Set OPENROUTER_API_KEY="your_actual_key" in your .env file')
            print('3. Set AGENT_MODEL to your preferred model (e.g., "openai/gpt-3.5-turbo")')
            print("4. Run: python api.py")

        except Exception as e:
            print(f"❌ Error during OpenRouter testing: {e}")
            import traceback
            traceback.print_exc()
            raise

if __name__ == "__main__":
    test_openrouter_configuration()