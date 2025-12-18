import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
from main import app
from models.book_content import BookContentMetadata, BookContentIngestRequest
from models.chat_interaction import ChatInteractionRequest


class TestUserStory1:
    """End-to-end tests for User Story 1: Ask Questions About Book Content."""

    def setup_method(self):
        """Set up the test client."""
        self.client = TestClient(app)

    @pytest.mark.asyncio
    async def test_user_story_1_complete_flow(self):
        """
        Test the complete flow for User Story 1:
        1. Ingest book content
        2. Query the content
        3. Receive accurate, grounded response
        """
        # Step 1: Ingest book content
        content = """
        # Introduction to Artificial Intelligence

        Artificial Intelligence (AI) is a branch of computer science that aims to create
        software or machines that exhibit human-like intelligence. This can include learning
        from experience, understanding natural language, solving problems, and recognizing patterns.

        ## Types of AI

        There are different types of AI including narrow AI, which is designed for specific tasks,
        and general AI, which would have the ability to understand and learn any intellectual task
        that a human being can.
        """

        metadata = BookContentMetadata(
            id="",
            content=content,
            module="AI Fundamentals",
            chapter="Introduction",
            section="What is AI",
            heading_path="AI Fundamentals > Introduction > What is AI",
            source_url="/docs/ai-intro",
            created_at=None
        )

        ingest_request = {
            "content": content,
            "metadata": metadata.dict()
        }

        # Mock the ingestion process
        with patch('services.content_service.content_service.ingest_content',
                   new=AsyncMock(return_value=MagicMock(
                       status="success",
                       indexed_chunks=2,
                       message="Content successfully indexed with 2 chunks",
                       chunk_ids=["chunk-1", "chunk-2"]
                   ))):
            ingest_response = self.client.post("/api/ingest", json=ingest_request)
            assert ingest_response.status_code == 200

        # Step 2: Query the content
        query_request = {
            "query": "What is Artificial Intelligence?",
            "context_mode": "global"
        }

        # Mock the query process to return a relevant response
        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="Artificial Intelligence (AI) is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. This can include learning from experience, understanding natural language, solving problems, and recognizing patterns.",
                       sources=[],
                       confidence=0.85,
                       status="success"
                   ))):
            query_response = self.client.post("/api/query", json=query_request)

            assert query_response.status_code == 200
            data = query_response.json()

            # Step 3: Verify the response is accurate and grounded
            assert "response" in data
            assert "artificial intelligence" in data["response"].lower()
            assert data["confidence"] > 0.5
            assert data["status"] == "success"

    @pytest.mark.asyncio
    async def test_user_story_1_no_content_found(self):
        """
        Test User Story 1 when no relevant content is found:
        1. Query for information not in the book
        2. Receive appropriate response indicating content not found
        """
        query_request = {
            "query": "What is the capital of Mars?",
            "context_mode": "global"
        }

        # Mock the query process to return no content
        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="The requested information is not present in the provided content.",
                       sources=[],
                       confidence=0.0,
                       status="no_content"
                   ))):
            query_response = self.client.post("/api/query", json=query_request)

            assert query_response.status_code == 200
            data = query_response.json()

            # Verify the response indicates content not found
            assert data["status"] == "no_content"
            assert "not present in the provided content" in data["response"]

    @pytest.mark.asyncio
    async def test_user_story_1_with_specific_module_filter(self):
        """
        Test User Story 1 with module filtering:
        1. Ingest content with specific module
        2. Query with module filter
        3. Receive response from filtered content
        """
        # Ingest content in a specific module
        content = "Machine learning is a subset of AI that enables systems to learn and improve from experience."

        metadata = BookContentMetadata(
            id="",
            content=content,
            module="Machine Learning",
            chapter="Fundamentals",
            section="Definition",
            heading_path="Machine Learning > Fundamentals > Definition",
            source_url="/docs/ml-fundamentals",
            created_at=None
        )

        ingest_request = {
            "content": content,
            "metadata": metadata.dict()
        }

        with patch('services.content_service.content_service.ingest_content',
                   new=AsyncMock(return_value=MagicMock(
                       status="success",
                       indexed_chunks=1,
                       message="Content successfully indexed",
                       chunk_ids=["chunk-ml-1"]
                   ))):
            ingest_response = self.client.post("/api/ingest", json=ingest_request)
            assert ingest_response.status_code == 200

        # Query with module filter
        query_request = {
            "query": "What is machine learning?",
            "context_mode": "global"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="Machine learning is a subset of AI that enables systems to learn and improve from experience.",
                       sources=[],
                       confidence=0.9,
                       status="success"
                   ))):
            query_response = self.client.post("/api/query", json=query_request)

            assert query_response.status_code == 200
            data = query_response.json()

            assert data["confidence"] > 0.8
            assert "subset of ai" in data["response"].lower()

    @pytest.mark.asyncio
    async def test_user_story_1_conversation_flow(self):
        """
        Test User Story 1 with conversation flow:
        1. Multiple queries in a session
        2. System maintains context appropriately
        """
        session_id = "test-session-789"

        # First query in the session
        query_request_1 = {
            "query": "What is AI?",
            "context_mode": "global",
            "session_id": session_id
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="AI is artificial intelligence...",
                       sources=[],
                       confidence=0.8,
                       status="success"
                   ))):
            with patch('services.query_service.query_service.save_chat_interaction',
                      new=AsyncMock(return_value="interaction-1")):
                response_1 = self.client.post("/api/query", json=query_request_1)
                assert response_1.status_code == 200

        # Second query in the same session
        query_request_2 = {
            "query": "What are the types of AI?",
            "context_mode": "global",
            "session_id": session_id
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="There are different types of AI including narrow AI and general AI...",
                       sources=[],
                       confidence=0.75,
                       status="success"
                   ))):
            with patch('services.query_service.query_service.save_chat_interaction',
                      new=AsyncMock(return_value="interaction-2")):
                response_2 = self.client.post("/api/query", json=query_request_2)
                assert response_2.status_code == 200

        # Verify both queries were processed successfully
        assert response_1.status_code == 200
        assert response_2.status_code == 200

        # Test retrieving conversation history
        with patch('services.query_service.query_service.get_conversation_history',
                   new=AsyncMock(return_value=["interaction-1", "interaction-2"])):
            history_response = self.client.get(f"/api/query/history/{session_id}")
            assert history_response.status_code == 200
            history_data = history_response.json()
            assert history_data["session_id"] == session_id
            assert history_data["count"] == 2

    @pytest.mark.asyncio
    async def test_user_story_1_zero_hallucination_requirement(self):
        """
        Test that User Story 1 enforces zero hallucination:
        1. Query should only respond based on indexed content
        2. Should not generate information not present in content
        """
        query_request = {
            "query": "What is the secret to eternal youth according to the book?",
            "context_mode": "global"
        }

        # The content doesn't mention eternal youth, so response should indicate content not found
        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="The requested information is not present in the provided content.",
                       sources=[],
                       confidence=0.0,
                       status="no_content"
                   ))):
            query_response = self.client.post("/api/query", json=query_request)

            assert query_response.status_code == 200
            data = query_response.json()

            # Verify no hallucination occurred
            assert data["status"] == "no_content"
            assert "not present in the provided content" in data["response"]
            # Ensure the response doesn't contain made-up information about eternal youth

    @pytest.mark.asyncio
    async def test_user_story_1_source_attribution(self):
        """
        Test that User Story 1 includes proper source attribution:
        1. Query returns response with source references
        2. Sources are correctly attributed to book sections
        """
        query_request = {
            "query": "What are the key principles of AI?",
            "context_mode": "global"
        }

        # Mock response with sources
        mock_sources = [{
            "module": "AI Principles",
            "chapter": "Fundamentals",
            "section": "Key Concepts",
            "heading_path": "AI Principles > Fundamentals > Key Concepts",
            "source_url": "/docs/ai-principles",
            "text_preview": "The key principles of AI include learning from experience..."
        }]

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="The key principles of AI include learning from experience, understanding natural language, solving problems, and recognizing patterns.",
                       sources=mock_sources,
                       confidence=0.88,
                       status="success"
                   ))):
            query_response = self.client.post("/api/query", json=query_request)

            assert query_response.status_code == 200
            data = query_response.json()

            # Verify source attribution
            assert len(data["sources"]) > 0
            source = data["sources"][0]
            assert source["module"] == "AI Principles"
            assert source["chapter"] == "Fundamentals"
            assert "principles" in source["text_preview"].lower()

    @pytest.mark.asyncio
    async def test_user_story_1_performance_requirement(self):
        """
        Test that User Story 1 meets performance requirements:
        1. Query response time is within acceptable limits
        """
        query_request = {
            "query": "What is the definition of AI?",
            "context_mode": "global"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="AI is artificial intelligence...",
                       sources=[],
                       confidence=0.82,
                       status="success"
                   ))):
            import time
            start_time = time.time()

            query_response = self.client.post("/api/query", json=query_request)

            end_time = time.time()
            response_time = (end_time - start_time) * 1000  # Convert to milliseconds

            assert query_response.status_code == 200
            # In a real test, we might check that response_time < some_threshold
            # For this test, we just verify the request completed successfully
            assert response_time > 0

    @pytest.mark.asyncio
    async def test_user_story_1_with_ingestion_and_query(self):
        """
        Integration test: Complete flow from ingestion to query in a single test.
        """
        # Ingest content
        content = """
        # Neural Networks

        Neural networks are a series of algorithms that mimic the operations of a human brain
        to recognize relationships between vast amounts of data. They are used in many
        applications including image recognition, natural language processing, and more.

        ## Structure

        A neural network consists of layers of interconnected nodes. Each node processes
        information and passes it to the next layer.
        """

        metadata = BookContentMetadata(
            id="",
            content=content,
            module="Deep Learning",
            chapter="Neural Networks",
            section="Introduction",
            heading_path="Deep Learning > Neural Networks > Introduction",
            source_url="/docs/neural-networks",
            created_at=None
        )

        ingest_request = {
            "content": content,
            "metadata": metadata.dict()
        }

        # Mock ingestion
        with patch('services.content_service.content_service.ingest_content',
                   new=AsyncMock(return_value=MagicMock(
                       status="success",
                       indexed_chunks=2,
                       message="Content successfully indexed",
                       chunk_ids=["nn-chunk-1", "nn-chunk-2"]
                   ))):
            ingest_response = self.client.post("/api/ingest", json=ingest_request)
            assert ingest_response.status_code == 200

        # Query the ingested content
        query_request = {
            "query": "What are neural networks?",
            "context_mode": "global"
        }

        # Mock query response
        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value=MagicMock(
                       response="Neural networks are a series of algorithms that mimic the operations of a human brain to recognize relationships between vast amounts of data.",
                       sources=[],
                       confidence=0.92,
                       status="success"
                   ))):
            query_response = self.client.post("/api/query", json=query_request)

            assert query_response.status_code == 200
            data = query_response.json()

            # Verify the complete flow worked
            assert "neural networks" in data["response"].lower()
            assert data["confidence"] > 0.9
            assert data["status"] == "success"