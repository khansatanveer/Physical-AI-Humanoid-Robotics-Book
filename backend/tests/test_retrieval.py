import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from typing import List
from services.retrieval_service import retrieval_service
from models.book_content import ContentSearchResult, ContentFilter


class TestRetrievalService:
    """Unit tests for the retrieval service."""

    @pytest.fixture
    def sample_search_results(self):
        """Sample search results for testing."""
        return [
            ContentSearchResult(
                id="test-id-1",
                content="This is the first test content chunk.",
                module="Test Module",
                chapter="Test Chapter",
                section="Test Section",
                heading_path="Test Module > Test Chapter > Test Section",
                source_url="/test/source/1",
                score=0.85
            ),
            ContentSearchResult(
                id="test-id-2",
                content="This is the second test content chunk.",
                module="Test Module",
                chapter="Test Chapter",
                section="Test Section 2",
                heading_path="Test Module > Test Chapter > Test Section 2",
                source_url="/test/source/2",
                score=0.78
            )
        ]

    @pytest.mark.asyncio
    async def test_retrieve_similar_content_basic(self, sample_search_results):
        """Test basic similar content retrieval."""
        query = "test query"

        with patch.object(retrieval_service.qdrant_service, 'search_similar',
                         new=AsyncMock(return_value=[
                             {
                                 "id": "test-id-1",
                                 "content": "This is the first test content chunk.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section",
                                 "heading_path": "Test Module > Test Chapter > Test Section",
                                 "source_url": "/test/source/1",
                                 "score": 0.85
                             }
                         ])):
            with patch.object(retrieval_service.embedding_utils, 'generate_single_embedding',
                             new=AsyncMock(return_value=[0.1, 0.2, 0.3])):

                results = await retrieval_service.retrieve_similar_content(query, top_k=1)

                assert len(results) == 1
                assert results[0].id == "test-id-1"
                assert results[0].content == "This is the first test content chunk."
                assert results[0].score == 0.85

    @pytest.mark.asyncio
    async def test_retrieve_similar_content_with_filters(self, sample_search_results):
        """Test similar content retrieval with filters."""
        query = "test query"
        filters = ContentFilter(module="Test Module", chapter="Test Chapter")

        with patch.object(retrieval_service.qdrant_service, 'search_similar',
                         new=AsyncMock(return_value=[
                             {
                                 "id": "test-id-1",
                                 "content": "Filtered content.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section",
                                 "heading_path": "Test Module > Test Chapter > Test Section",
                                 "source_url": "/test/source/1",
                                 "score": 0.90
                             }
                         ])):
            with patch.object(retrieval_service.embedding_utils, 'generate_single_embedding',
                             new=AsyncMock(return_value=[0.1, 0.2, 0.3])):

                results = await retrieval_service.retrieve_similar_content(
                    query, top_k=1, filters=filters
                )

                assert len(results) == 1
                assert results[0].content == "Filtered content."
                assert results[0].module == "Test Module"
                assert results[0].chapter == "Test Chapter"

    @pytest.mark.asyncio
    async def test_retrieve_similar_content_with_min_score(self):
        """Test similar content retrieval with minimum score threshold."""
        query = "test query"

        with patch.object(retrieval_service.qdrant_service, 'search_similar',
                         new=AsyncMock(return_value=[
                             {
                                 "id": "high-score-id",
                                 "content": "High score content.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section",
                                 "heading_path": "Test Module > Test Chapter > Test Section",
                                 "source_url": "/test/source/1",
                                 "score": 0.90
                             },
                             {
                                 "id": "low-score-id",
                                 "content": "Low score content.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section",
                                 "heading_path": "Test Module > Test Chapter > Test Section",
                                 "source_url": "/test/source/2",
                                 "score": 0.30  # Below threshold
                             }
                         ])):
            with patch.object(retrieval_service.embedding_utils, 'generate_single_embedding',
                             new=AsyncMock(return_value=[0.1, 0.2, 0.3])):

                results = await retrieval_service.retrieve_similar_content(
                    query, top_k=5, min_score=0.5
                )

                assert len(results) == 1  # Only the high-score result should remain
                assert results[0].id == "high-score-id"
                assert results[0].score >= 0.5

    @pytest.mark.asyncio
    async def test_retrieve_by_source_url(self):
        """Test retrieving content by source URL."""
        source_url = "/test/source/1"

        with patch.object(retrieval_service, 'retrieve_similar_content',
                         new=AsyncMock(return_value=[
                             ContentSearchResult(
                                 id="test-id-1",
                                 content="Content from source.",
                                 module="Test Module",
                                 chapter="Test Chapter",
                                 section="Test Section",
                                 heading_path="Test Module > Test Chapter > Test Section",
                                 source_url=source_url,
                                 score=0.85
                             )
                         ])):

                results = await retrieval_service.retrieve_by_source_url(source_url)

                assert len(results) == 1
                assert results[0].source_url == source_url

    @pytest.mark.asyncio
    async def test_retrieve_by_metadata(self):
        """Test retrieving content by metadata filters."""
        module = "Test Module"
        chapter = "Test Chapter"

        with patch.object(retrieval_service, 'retrieve_similar_content',
                         new=AsyncMock(return_value=[
                             ContentSearchResult(
                                 id="test-id-1",
                                 content="Content with metadata.",
                                 module=module,
                                 chapter=chapter,
                                 section="Test Section",
                                 heading_path="Test Module > Test Chapter > Test Section",
                                 source_url="/test/source/1",
                                 score=0.85
                             )
                         ])):

                results = await retrieval_service.retrieve_by_metadata(
                    module=module, chapter=chapter
                )

                assert len(results) == 1
                assert results[0].module == module
                assert results[0].chapter == chapter

    @pytest.mark.asyncio
    async def test_multi_query_retrieval(self):
        """Test retrieval with multiple related queries."""
        queries = ["query one", "query two"]
        mock_results = [
            ContentSearchResult(
                id="result-1",
                content="Result for query one.",
                module="Test Module",
                chapter="Test Chapter",
                section="Test Section",
                heading_path="Test Module > Test Chapter > Test Section",
                source_url="/test/source/1",
                score=0.85
            ),
            ContentSearchResult(
                id="result-2",
                content="Result for query two.",
                module="Test Module",
                chapter="Test Chapter",
                section="Test Section 2",
                heading_path="Test Module > Test Chapter > Test Section 2",
                source_url="/test/source/2",
                score=0.78
            )
        ]

        with patch.object(retrieval_service, 'retrieve_similar_content',
                         side_effect=[mock_results[:1], mock_results[1:]]):

                results = await retrieval_service.multi_query_retrieval(
                    queries, top_k_per_query=1
                )

                assert len(results) == 2
                assert results[0].content == "Result for query one."
                assert results[1].content == "Result for query two."

    @pytest.mark.asyncio
    async def test_find_content_chunks_by_similarity_threshold(self):
        """Test finding content chunks by similarity threshold."""
        query = "test query"

        with patch.object(retrieval_service.qdrant_service, 'search_similar',
                         new=AsyncMock(return_value=[
                             {
                                 "id": "high-score-id",
                                 "content": "High similarity content.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section",
                                 "heading_path": "Test Module > Test Chapter > Test Section",
                                 "source_url": "/test/source/1",
                                 "score": 0.90
                             },
                             {
                                 "id": "medium-score-id",
                                 "content": "Medium similarity content.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section 2",
                                 "heading_path": "Test Module > Test Chapter > Test Section 2",
                                 "source_url": "/test/source/2",
                                 "score": 0.60
                             },
                             {
                                 "id": "low-score-id",
                                 "content": "Low similarity content.",
                                 "module": "Test Module",
                                 "chapter": "Test Chapter",
                                 "section": "Test Section 3",
                                 "heading_path": "Test Module > Test Chapter > Test Section 3",
                                 "source_url": "/test/source/3",
                                 "score": 0.30
                             }
                         ])):
            with patch.object(retrieval_service.embedding_utils, 'generate_single_embedding',
                             new=AsyncMock(return_value=[0.1, 0.2, 0.3])):

                results = await retrieval_service.find_content_chunks_by_similarity_threshold(
                    query, similarity_threshold=0.5, max_results=5
                )

                # Should only return results with score >= 0.5
                assert len(results) == 2  # high and medium scores
                assert all(r.score >= 0.5 for r in results)

    @pytest.mark.asyncio
    async def test_health_check_success(self):
        """Test health check when all dependencies are healthy."""
        with patch.object(retrieval_service.qdrant_service, 'health_check',
                         new=AsyncMock(return_value=True)):
            with patch.object(retrieval_service.cohere_service, 'health_check',
                             new=AsyncMock(return_value=True)):

                is_healthy = await retrieval_service.health_check()

                assert is_healthy is True

    @pytest.mark.asyncio
    async def test_health_check_failure(self):
        """Test health check when one dependency is unhealthy."""
        with patch.object(retrieval_service.qdrant_service, 'health_check',
                         new=AsyncMock(return_value=False)):
            with patch.object(retrieval_service.cohere_service, 'health_check',
                             new=AsyncMock(return_value=True)):

                is_healthy = await retrieval_service.health_check()

                assert is_healthy is False

    @pytest.mark.asyncio
    async def test_validate_retrieval_success(self):
        """Test retrieval validation when expected items are retrieved."""
        query = "test query"
        expected_ids = ["expected-id-1", "expected-id-2"]

        mock_results = [
            ContentSearchResult(
                id="expected-id-1",
                content="Content 1",
                module="Test",
                chapter="Test",
                section="Test",
                heading_path="Test > Test > Test",
                source_url="/test/1",
                score=0.9
            ),
            ContentSearchResult(
                id="expected-id-2",
                content="Content 2",
                module="Test",
                chapter="Test",
                section="Test",
                heading_path="Test > Test > Test",
                source_url="/test/2",
                score=0.8
            )
        ]

        with patch.object(retrieval_service, 'retrieve_similar_content',
                         new=AsyncMock(return_value=mock_results)):

                is_valid, message, retrieved_ids = await retrieval_service.validate_retrieval(
                    query, expected_ids, top_k=5
                )

                assert is_valid is True
                assert "2/2 expected items" in message
                assert set(retrieved_ids) == set(expected_ids)

    @pytest.mark.asyncio
    async def test_validate_retrieval_partial_success(self):
        """Test retrieval validation when only some expected items are retrieved."""
        query = "test query"
        expected_ids = ["expected-id-1", "expected-id-2", "expected-id-3"]

        mock_results = [
            ContentSearchResult(
                id="expected-id-1",
                content="Content 1",
                module="Test",
                chapter="Test",
                section="Test",
                heading_path="Test > Test > Test",
                source_url="/test/1",
                score=0.9
            ),
            # Missing expected-id-2
            ContentSearchResult(
                id="expected-id-3",
                content="Content 3",
                module="Test",
                chapter="Test",
                section="Test",
                heading_path="Test > Test > Test",
                source_url="/test/3",
                score=0.7
            )
        ]

        with patch.object(retrieval_service, 'retrieve_similar_content',
                         new=AsyncMock(return_value=mock_results)):

                is_valid, message, retrieved_ids = await retrieval_service.validate_retrieval(
                    query, expected_ids, top_k=5
                )

                # Should be invalid since only 2 out of 3 expected items were retrieved (< 50%)
                assert is_valid is False
                assert "2/3 expected items" in message
                assert len(retrieved_ids) == 2

    @pytest.mark.asyncio
    async def test_retrieve_similar_content_empty_query(self):
        """Test retrieval with empty query."""
        with patch.object(retrieval_service.embedding_utils, 'generate_single_embedding',
                         new=AsyncMock(return_value=[])):

                results = await retrieval_service.retrieve_similar_content("", top_k=5)

                # Should return empty list when query embedding fails
                assert len(results) == 0

    @pytest.mark.asyncio
    async def test_retrieve_similar_content_exception_handling(self):
        """Test retrieval handles exceptions properly."""
        query = "test query"

        with patch.object(retrieval_service.embedding_utils, 'generate_single_embedding',
                         new=AsyncMock(side_effect=Exception("Embedding error"))):

                # Should handle the exception and return empty results
                results = await retrieval_service.retrieve_similar_content(query, top_k=5)

                assert len(results) == 0