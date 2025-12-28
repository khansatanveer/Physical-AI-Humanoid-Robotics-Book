"""
Unit tests for the retrieve.py functionality.
"""
import unittest
from unittest.mock import Mock, patch
import sys
import os

# Add the backend directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from retrieve import retrieve_content


class TestRetrieveContent(unittest.TestCase):
    """Unit tests for the retrieve_content function."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the embeddings generator and storage manager
        self.mock_embeddings_gen = Mock()
        self.mock_storage_manager = Mock()

        # Mock embedding generation
        self.mock_embeddings_gen.generate_embedding_for_chunk.return_value = [0.1, 0.2, 0.3]

        # Mock search results
        self.mock_storage_manager.search_similar.return_value = [
            {
                'id': 'test_id_1',
                'score': 0.9,
                'payload': {
                    'content': 'Test content 1',
                    'url': 'https://test.com/page1',
                    'title': 'Test Title 1',
                    'headings': ['Heading 1'],
                    'metadata': {'content_hash': 'hash1'}
                }
            }
        ]

    def test_retrieve_content_basic(self):
        """Test basic retrieval functionality."""
        query = "test query"
        top_k = 5

        results = retrieve_content(
            query=query,
            top_k=top_k,
            embeddings_gen=self.mock_embeddings_gen,
            storage_manager=self.mock_storage_manager
        )

        # Verify the embeddings were generated
        self.mock_embeddings_gen.generate_embedding_for_chunk.assert_called_once_with({'content': query})

        # Verify the search was performed
        self.mock_storage_manager.search_similar.assert_called_once_with([0.1, 0.2, 0.3], limit=top_k)

        # Verify we got results back
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].content, 'Test content 1')
        self.assertEqual(results[0].url, 'https://test.com/page1')

    def test_retrieve_content_with_empty_query(self):
        """Test retrieval with empty query raises ValueError."""
        with self.assertRaises(ValueError) as context:
            retrieve_content(
                query="",
                top_k=5,
                embeddings_gen=self.mock_embeddings_gen,
                storage_manager=self.mock_storage_manager
            )

        self.assertIn("Query validation failed", str(context.exception))

    def test_retrieve_content_with_short_query(self):
        """Test retrieval with very short query raises ValueError."""
        with self.assertRaises(ValueError) as context:
            retrieve_content(
                query="a",
                top_k=5,
                embeddings_gen=self.mock_embeddings_gen,
                storage_manager=self.mock_storage_manager
            )

        self.assertIn("Query validation failed", str(context.exception))

    def test_retrieve_content_with_invalid_top_k(self):
        """Test retrieval with invalid top_k raises ValueError."""
        with self.assertRaises(ValueError) as context:
            retrieve_content(
                query="valid query",
                top_k=0,
                embeddings_gen=self.mock_embeddings_gen,
                storage_manager=self.mock_storage_manager
            )

        self.assertIn("top_k validation failed", str(context.exception))

    def test_retrieve_content_with_large_top_k(self):
        """Test retrieval with too large top_k raises ValueError."""
        with self.assertRaises(ValueError) as context:
            retrieve_content(
                query="valid query",
                top_k=101,
                embeddings_gen=self.mock_embeddings_gen,
                storage_manager=self.mock_storage_manager
            )

        self.assertIn("top_k validation failed", str(context.exception))


if __name__ == '__main__':
    unittest.main()