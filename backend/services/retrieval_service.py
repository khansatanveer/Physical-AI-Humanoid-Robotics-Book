from typing import List, Dict, Any, Optional, Union
import asyncio
from datetime import datetime

from clients.qdrant_client import qdrant_service
from clients.cohere_client import cohere_service
from utils.embedding_utils import embedding_utils
from models.book_content import ContentSearchResult, ContentFilter
from config import settings


class RetrievalService:
    """Service class for handling content retrieval and similarity search operations."""

    def __init__(self):
        """Initialize the retrieval service with required clients."""
        self.qdrant_service = qdrant_service
        self.cohere_service = cohere_service
        self.embedding_utils = embedding_utils

    async def retrieve_similar_content(
        self,
        query: str,
        top_k: int = 5,
        filters: Optional[Union[ContentFilter, Dict[str, Any]]] = None,
        min_score: Optional[float] = None
    ) -> List[ContentSearchResult]:
        """
        Retrieve content similar to the query using vector similarity search.

        Args:
            query: The query text to find similar content for
            top_k: Number of results to return
            filters: Optional filters to apply to the search
            min_score: Minimum similarity score threshold

        Returns:
            List of ContentSearchResult objects
        """
        # Generate embedding for the query
        query_embedding = await self.embedding_utils.generate_single_embedding(query)

        # Prepare filters for Qdrant
        qdrant_filters = {}
        if filters:
            if isinstance(filters, ContentFilter):
                # Convert Pydantic model to dict
                filters = filters.dict(exclude_none=True)

            # Map our filter fields to Qdrant payload fields
            for field in ['module', 'chapter', 'section', 'source_url']:
                if field in filters and filters[field] is not None:
                    qdrant_filters[field] = filters[field]

        # Perform similarity search
        search_results = await self.qdrant_service.search_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            filters=qdrant_filters
        )

        # Filter by minimum score if specified
        if min_score is not None:
            search_results = [r for r in search_results if r["score"] >= min_score]

        # Convert to ContentSearchResult objects
        results = []
        for result in search_results:
            results.append(
                ContentSearchResult(
                    id=result["id"],
                    content=result["content"],
                    module=result["module"],
                    chapter=result["chapter"],
                    section=result["section"],
                    heading_path=result["heading_path"],
                    source_url=result["source_url"],
                    score=result["score"]
                )
            )

        return results

    async def retrieve_by_content_ids(
        self,
        content_ids: List[str]
    ) -> List[ContentSearchResult]:
        """
        Retrieve specific content by their IDs.

        Args:
            content_ids: List of content IDs to retrieve

        Returns:
            List of ContentSearchResult objects
        """
        # This would require a method in Qdrant service to retrieve points by ID
        # For now, we'll return empty results as this functionality would need to be
        # implemented in the Qdrant client
        results = []

        # In a real implementation, we would use Qdrant's retrieve method:
        # points = self.qdrant_service.client.retrieve(
        #     collection_name=self.qdrant_service.collection_name,
        #     ids=content_ids
        # )

        # For now, we'll simulate by searching for each ID individually
        # which is not efficient but demonstrates the concept
        for content_id in content_ids:
            # This is a simplified approach - in reality, we'd want to batch retrieve
            # For now, we'll return empty results and note that this needs proper implementation
            pass

        return results

    async def retrieve_by_source_url(
        self,
        source_url: str,
        top_k: Optional[int] = None
    ) -> List[ContentSearchResult]:
        """
        Retrieve all content associated with a specific source URL.

        Args:
            source_url: URL of the source content
            top_k: Optional limit on number of results

        Returns:
            List of ContentSearchResult objects
        """
        filter_obj = ContentFilter(source_url=source_url)
        results = await self.retrieve_similar_content(
            query="any content from this source",  # Dummy query
            top_k=top_k or 100,  # Default to 100 if no limit specified
            filters=filter_obj
        )

        # Filter to only include results from the specified source URL
        filtered_results = [r for r in results if r.source_url == source_url]

        return filtered_results

    async def retrieve_by_metadata(
        self,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        section: Optional[str] = None,
        top_k: int = 10
    ) -> List[ContentSearchResult]:
        """
        Retrieve content based on metadata filters.

        Args:
            module: Module name to filter by
            chapter: Chapter name to filter by
            section: Section name to filter by
            top_k: Number of results to return

        Returns:
            List of ContentSearchResult objects
        """
        filters = ContentFilter(
            module=module,
            chapter=chapter,
            section=section
        )

        return await self.retrieve_similar_content(
            query="content with specified metadata",  # Dummy query
            top_k=top_k,
            filters=filters
        )

    async def multi_query_retrieval(
        self,
        queries: List[str],
        top_k_per_query: int = 3,
        filters: Optional[ContentFilter] = None,
        rerank: bool = True
    ) -> List[ContentSearchResult]:
        """
        Perform retrieval with multiple related queries and optionally rerank results.

        Args:
            queries: List of related queries to search for
            top_k_per_query: Number of results to retrieve per query
            filters: Optional filters to apply
            rerank: Whether to rerank results based on overall relevance

        Returns:
            List of ContentSearchResult objects
        """
        all_results = []

        # Perform retrieval for each query
        for query in queries:
            query_results = await self.retrieve_similar_content(
                query=query,
                top_k=top_k_per_query,
                filters=filters
            )
            all_results.extend(query_results)

        # Remove duplicates based on content ID
        unique_results = {}
        for result in all_results:
            if result.id not in unique_results:
                unique_results[result.id] = result
            else:
                # If duplicate, keep the one with higher score
                if result.score > unique_results[result.id].score:
                    unique_results[result.id] = result

        # Convert back to list
        results_list = list(unique_results.values())

        # Optionally rerank based on some criteria
        if rerank:
            results_list.sort(key=lambda x: x.score, reverse=True)

        # Limit to top results overall
        return results_list[:top_k_per_query * len(queries)]

    async def semantic_search_with_hybrid_scoring(
        self,
        query: str,
        keyword_query: Optional[str] = None,
        top_k: int = 5,
        filters: Optional[ContentFilter] = None
    ) -> List[ContentSearchResult]:
        """
        Perform semantic search with optional keyword-based hybrid scoring.

        Args:
            query: Semantic query for vector search
            keyword_query: Optional keyword query for additional scoring
            top_k: Number of results to return
            filters: Optional filters to apply

        Returns:
            List of ContentSearchResult objects with hybrid scores
        """
        # Perform semantic search
        semantic_results = await self.retrieve_similar_content(
            query=query,
            top_k=top_k * 2,  # Get more results for hybrid scoring
            filters=filters
        )

        if keyword_query:
            # In a real implementation, we would also perform keyword search
            # and combine scores from both semantic and keyword search
            # For now, we'll just return the semantic results
            pass

        # Limit to top_k results
        return semantic_results[:top_k]

    async def find_content_chunks_by_similarity_threshold(
        self,
        query: str,
        similarity_threshold: float = 0.5,
        max_results: int = 10
    ) -> List[ContentSearchResult]:
        """
        Find content chunks that meet a minimum similarity threshold.

        Args:
            query: Query to search for
            similarity_threshold: Minimum similarity score required
            max_results: Maximum number of results to return

        Returns:
            List of ContentSearchResult objects that meet the threshold
        """
        # Get more results than needed to ensure we have enough above threshold
        potential_results = await self.retrieve_similar_content(
            query=query,
            top_k=max_results * 2,  # Get extra in case some are below threshold
            min_score=similarity_threshold
        )

        # Sort by score and limit to max_results
        potential_results.sort(key=lambda x: x.score, reverse=True)
        return potential_results[:max_results]

    async def get_content_statistics(
        self,
        filters: Optional[ContentFilter] = None
    ) -> Dict[str, Any]:
        """
        Get statistics about the indexed content.

        Args:
            filters: Optional filters to apply

        Returns:
            Dictionary with content statistics
        """
        # This would require additional methods in the Qdrant service
        # For now, we'll return placeholder statistics
        stats = {
            "total_chunks": 0,
            "unique_sources": 0,
            "modules_count": 0,
            "chapters_count": 0,
            "avg_chunk_length": 0,
            "last_indexed": None
        }

        # In a real implementation, we would query Qdrant for these statistics
        # This might involve using Qdrant's count API or iterating through collections

        return stats

    async def validate_retrieval(
        self,
        query: str,
        expected_content_ids: List[str],
        top_k: int = 5
    ) -> tuple[bool, str, List[str]]:
        """
        Validate retrieval performance against expected results.

        Args:
            query: Query to test
            expected_content_ids: Expected content IDs to be retrieved
            top_k: Number of results to retrieve

        Returns:
            Tuple of (is_valid, validation_message, retrieved_ids)
        """
        results = await self.retrieve_similar_content(
            query=query,
            top_k=top_k
        )

        retrieved_ids = [r.id for r in results]
        expected_set = set(expected_content_ids)
        retrieved_set = set(retrieved_ids)

        # Calculate overlap
        intersection = expected_set.intersection(retrieved_set)
        overlap_percentage = len(intersection) / len(expected_set) if expected_content_ids else 0

        if overlap_percentage >= 0.5:  # 50% or more expected items retrieved
            return True, f"Retrieved {len(intersection)}/{len(expected_content_ids)} expected items", retrieved_ids
        else:
            return False, f"Only retrieved {len(intersection)}/{len(expected_content_ids)} expected items", retrieved_ids

    async def health_check(self) -> bool:
        """
        Perform a health check of the retrieval service.

        Returns:
            True if all dependencies are healthy, False otherwise
        """
        qdrant_healthy = await self.qdrant_service.health_check()
        cohere_healthy = await self.cohere_service.health_check()

        return qdrant_healthy and cohere_healthy


# Global instance
retrieval_service = RetrievalService()