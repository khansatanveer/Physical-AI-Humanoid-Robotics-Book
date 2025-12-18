import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch
from main import app
from models.chat_interaction import ChatInteractionRequest, SelectionQueryRequest


class TestQueryAPI:
    """Integration tests for the query API endpoints."""

    def setup_method(self):
        """Set up the test client."""
        self.client = TestClient(app)

    @pytest.mark.asyncio
    async def test_query_endpoint_basic(self):
        """Test the basic query endpoint functionality."""
        request_data = {
            "query": "What is AI?",
            "context_mode": "global"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value={
                       "response": "AI is artificial intelligence...",
                       "sources": [],
                       "confidence": 0.8,
                       "status": "success"
                   })):
            response = self.client.post("/api/query", json=request_data)

            assert response.status_code == 200
            data = response.json()
            assert "response" in data
            assert "sources" in data
            assert "confidence" in data
            assert "status" in data

    @pytest.mark.asyncio
    async def test_query_endpoint_missing_query(self):
        """Test the query endpoint with missing query."""
        request_data = {
            "context_mode": "global"
        }

        response = self.client.post("/api/query", json=request_data)

        assert response.status_code == 422  # Validation error

    @pytest.mark.asyncio
    async def test_query_endpoint_empty_query(self):
        """Test the query endpoint with empty query."""
        request_data = {
            "query": "",
            "context_mode": "global"
        }

        response = self.client.post("/api/query", json=request_data)

        assert response.status_code == 400  # Bad request

    @pytest.mark.asyncio
    async def test_query_endpoint_invalid_context_mode(self):
        """Test the query endpoint with invalid context mode."""
        request_data = {
            "query": "Test query",
            "context_mode": "invalid_mode"
        }

        response = self.client.post("/api/query", json=request_data)

        assert response.status_code == 422  # Validation error

    @pytest.mark.asyncio
    async def test_selection_query_endpoint_basic(self):
        """Test the selection query endpoint functionality."""
        request_data = {
            "query": "What does this text mean?",
            "selected_text": "This is the selected text that should be used for context.",
            "context_mode": "selection"
        }

        with patch('services.query_service.query_service.process_selection_query',
                   new=AsyncMock(return_value={
                       "response": "The selected text means...",
                       "sources": [],
                       "confidence": 0.75,
                       "status": "success"
                   })):
            response = self.client.post("/api/query/selection", json=request_data)

            assert response.status_code == 200
            data = response.json()
            assert "response" in data
            assert "sources" in data
            assert "confidence" in data
            assert "status" in data

    @pytest.mark.asyncio
    async def test_selection_query_endpoint_missing_selected_text(self):
        """Test the selection query endpoint with missing selected text."""
        request_data = {
            "query": "What does this mean?",
            "context_mode": "selection"
        }

        response = self.client.post("/api/query/selection", json=request_data)

        assert response.status_code == 422  # Validation error

    @pytest.mark.asyncio
    async def test_selection_query_endpoint_empty_selected_text(self):
        """Test the selection query endpoint with empty selected text."""
        request_data = {
            "query": "What does this mean?",
            "selected_text": "",
            "context_mode": "selection"
        }

        response = self.client.post("/api/query/selection", json=request_data)

        assert response.status_code == 400  # Bad request

    @pytest.mark.asyncio
    async def test_selection_query_endpoint_wrong_context_mode(self):
        """Test the selection query endpoint with wrong context mode."""
        request_data = {
            "query": "What does this mean?",
            "selected_text": "Selected text",
            "context_mode": "global"  # Should be "selection"
        }

        response = self.client.post("/api/query/selection", json=request_data)

        assert response.status_code == 400  # Bad request

    @pytest.mark.asyncio
    async def test_filtered_query_endpoint(self):
        """Test the filtered query endpoint functionality."""
        request_data = {
            "query": "What is machine learning?",
            "context_mode": "global"
        }
        filters = {
            "module": "AI Fundamentals",
            "chapter": "Introduction"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value={
                       "response": "Machine learning is...",
                       "sources": [],
                       "confidence": 0.85,
                       "status": "success"
                   })):
            response = self.client.post("/api/query/filtered",
                                      json={**request_data, "filters": filters})

            assert response.status_code == 200
            data = response.json()
            assert "response" in data

    @pytest.mark.asyncio
    async def test_conversation_history_endpoint(self):
        """Test the conversation history endpoint."""
        session_id = "test-session-123"

        with patch('services.query_service.query_service.get_conversation_history',
                   new=AsyncMock(return_value=[])):
            response = self.client.get(f"/api/query/history/{session_id}")

            assert response.status_code == 200
            data = response.json()
            assert "session_id" in data
            assert data["session_id"] == session_id
            assert "history" in data
            assert "count" in data

    @pytest.mark.asyncio
    async def test_conversation_history_endpoint_not_found(self):
        """Test the conversation history endpoint with non-existent session."""
        session_id = "non-existent-session"

        with patch('services.query_service.query_service.get_conversation_history',
                   new=AsyncMock(side_effect=Exception("Session not found"))):
            response = self.client.get(f"/api/query/history/{session_id}")

            assert response.status_code == 500  # Internal server error

    @pytest.mark.asyncio
    async def test_get_all_source_urls(self):
        """Test the get all source URLs endpoint."""
        expected_urls = ["/docs/intro", "/docs/tutorial", "/docs/api"]

        with patch('services.content_service.content_service.get_all_source_urls',
                   new=AsyncMock(return_value=expected_urls)):
            response = self.client.get("/api/query/sources")

            assert response.status_code == 200
            data = response.json()
            assert "source_urls" in data
            assert data["count"] == len(expected_urls)

    @pytest.mark.asyncio
    async def test_query_health_check(self):
        """Test the query health check endpoint."""
        with patch('services.query_service.query_service.health_check',
                   new=AsyncMock(return_value=True)):
            response = self.client.get("/api/query/health")

            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "healthy"
            assert data["service"] == "query_service"
            assert "timestamp" in data

    @pytest.mark.asyncio
    async def test_query_health_check_unhealthy(self):
        """Test the query health check endpoint when unhealthy."""
        with patch('services.query_service.query_service.health_check',
                   new=AsyncMock(return_value=False)):
            response = self.client.get("/api/query/health")

            assert response.status_code == 200  # Health check returns 200 with status
            data = response.json()
            assert data["status"] == "unhealthy"

    @pytest.mark.asyncio
    async def test_query_endpoint_error_handling(self):
        """Test that the query endpoint handles errors properly."""
        request_data = {
            "query": "Problematic query",
            "context_mode": "global"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(side_effect=Exception("Processing error"))):
            response = self.client.post("/api/query", json=request_data)

            assert response.status_code == 500
            data = response.json()
            assert "detail" in data

    @pytest.mark.asyncio
    async def test_selection_query_endpoint_error_handling(self):
        """Test that the selection query endpoint handles errors properly."""
        request_data = {
            "query": "Problematic selection query",
            "selected_text": "Some selected text",
            "context_mode": "selection"
        }

        with patch('services.query_service.query_service.process_selection_query',
                   new=AsyncMock(side_effect=Exception("Processing error"))):
            response = self.client.post("/api/query/selection", json=request_data)

            assert response.status_code == 500
            data = response.json()
            assert "detail" in data

    @pytest.mark.asyncio
    async def test_query_with_session_id(self):
        """Test the query endpoint with session ID."""
        request_data = {
            "query": "What is AI?",
            "context_mode": "global",
            "session_id": "session-123"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value={
                       "response": "AI is artificial intelligence...",
                       "sources": [],
                       "confidence": 0.8,
                       "status": "success"
                   })):
            with patch('services.query_service.query_service.save_chat_interaction',
                      new=AsyncMock(return_value="interaction-123")):
                response = self.client.post("/api/query", json=request_data)

                assert response.status_code == 200
                data = response.json()
                assert "response" in data

    @pytest.mark.asyncio
    async def test_selection_query_with_session_id(self):
        """Test the selection query endpoint with session ID."""
        request_data = {
            "query": "What does this mean?",
            "selected_text": "Selected context",
            "context_mode": "selection",
            "session_id": "session-456"
        }

        with patch('services.query_service.query_service.process_selection_query',
                   new=AsyncMock(return_value={
                       "response": "The selected text means...",
                       "sources": [],
                       "confidence": 0.75,
                       "status": "success"
                   })):
            with patch('services.query_service.query_service.save_chat_interaction',
                      new=AsyncMock(return_value="interaction-456")):
                response = self.client.post("/api/query/selection", json=request_data)

                assert response.status_code == 200
                data = response.json()
                assert "response" in data

    @pytest.mark.asyncio
    async def test_filtered_query_with_parameters(self):
        """Test the filtered query endpoint with additional parameters."""
        request_data = {
            "query": "What is deep learning?",
            "context_mode": "global"
        }
        filters = {
            "module": "Advanced AI",
            "chapter": "Neural Networks"
        }

        with patch('services.query_service.query_service.process_global_query',
                   new=AsyncMock(return_value={
                       "response": "Deep learning is...",
                       "sources": [],
                       "confidence": 0.9,
                       "status": "success"
                   })):
            response = self.client.post(
                "/api/query/filtered?top_k=3",
                json={**request_data, "filters": filters}
            )

            assert response.status_code == 200
            data = response.json()
            assert "response" in data