from typing import List, Optional, Dict, Any
import asyncio
from datetime import datetime
import json

from models.book_content import SourceReference, ContentSearchResult
from models.chat_interaction import QueryResponse
from services.generation_service import generation_service
from services.retrieval_service import retrieval_service
from config import settings


class ResponseService:
    """Service class for handling response formatting and source attribution."""

    def __init__(self):
        """Initialize the response service."""
        self.generation_service = generation_service
        self.retrieval_service = retrieval_service

    async def format_query_response(
        self,
        response_text: str,
        sources: List[SourceReference],
        confidence: float,
        status: str = "success"
    ) -> QueryResponse:
        """
        Format a query response with proper structure and source attribution.

        Args:
            response_text: The generated response text
            sources: List of sources used in the response
            confidence: Confidence level in the response
            status: Status of the query processing

        Returns:
            QueryResponse object with formatted response
        """
        return QueryResponse(
            response=response_text,
            sources=sources,
            confidence=confidence,
            status=status
        )

    async def create_attribution_from_search_results(
        self,
        search_results: List[ContentSearchResult]
    ) -> List[SourceReference]:
        """
        Create source attribution from search results.

        Args:
            search_results: List of search results to convert to source references

        Returns:
            List of SourceReference objects
        """
        sources = []
        for result in search_results:
            source = SourceReference(
                module=result.module,
                chapter=result.chapter,
                section=result.section,
                heading_path=result.heading_path,
                source_url=result.source_url,
                text_preview=result.content[:200] + "..." if len(result.content) > 200 else result.content
            )
            sources.append(source)

        return sources

    async def format_response_with_attribution(
        self,
        response_text: str,
        sources: List[SourceReference],
        include_detailed_attribution: bool = True
    ) -> str:
        """
        Format response text with source attribution.

        Args:
            response_text: The response text to format
            sources: List of sources to attribute
            include_detailed_attribution: Whether to include detailed source information

        Returns:
            Formatted response string with attribution
        """
        formatted_response = response_text

        if sources and include_detailed_attribution:
            formatted_response += "\n\nSources referenced:\n"
            for i, source in enumerate(sources, 1):
                formatted_response += f"{i}. {source.module} > {source.chapter} > {source.section}\n"
                if source.source_url:
                    formatted_response += f"   Source: {source.source_url}\n"
                if source.text_preview:
                    formatted_response += f"   Excerpt: {source.text_preview[:100]}...\n"

        return formatted_response

    async def generate_attribution_summary(
        self,
        sources: List[SourceReference]
    ) -> Dict[str, Any]:
        """
        Generate a summary of source attribution.

        Args:
            sources: List of sources to summarize

        Returns:
            Dictionary with attribution summary
        """
        if not sources:
            return {
                "total_sources": 0,
                "modules": [],
                "chapters": [],
                "sections": [],
                "source_urls": []
            }

        # Extract unique values
        modules = list(set(s.module for s in sources if s.module))
        chapters = list(set(s.chapter for s in sources if s.chapter))
        sections = list(set(s.section for s in sources if s.section))
        source_urls = list(set(s.source_url for s in sources if s.source_url))

        return {
            "total_sources": len(sources),
            "modules": modules,
            "chapters": chapters,
            "sections": sections,
            "source_urls": source_urls
        }

    async def validate_source_attribution(
        self,
        response: str,
        sources: List[SourceReference],
        original_context: List[Dict[str, str]]
    ) -> tuple[bool, str]:
        """
        Validate that the response properly attributes sources.

        Args:
            response: The generated response
            sources: List of sources that should be attributed
            original_context: Original context chunks used for generation

        Returns:
            Tuple of (is_valid, validation_message)
        """
        if not sources:
            return True, "No sources to attribute"

        # Check if the response contains any of the source identifiers
        response_lower = response.lower()
        attributed_sources = 0

        for source in sources:
            # Check if any significant part of the source info appears in the response
            if (source.module.lower() in response_lower or
                source.chapter.lower() in response_lower or
                source.section.lower() in response_lower or
                source.source_url.lower() in response_lower):
                attributed_sources += 1

        if attributed_sources == 0:
            return False, "Response does not appear to attribute any sources"

        return True, f"Response attributes {attributed_sources}/{len(sources)} sources"

    async def create_response_metadata(
        self,
        query: str,
        response: str,
        sources: List[SourceReference],
        retrieval_time: Optional[float] = None,
        generation_time: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Create metadata for a response.

        Args:
            query: The original query
            response: The generated response
            sources: List of sources used
            retrieval_time: Time taken for retrieval
            generation_time: Time taken for generation

        Returns:
            Dictionary with response metadata
        """
        attribution_summary = await self.generate_attribution_summary(sources)

        return {
            "timestamp": datetime.utcnow().isoformat(),
            "query_length": len(query),
            "response_length": len(response),
            "sources_count": len(sources),
            "attribution_summary": attribution_summary,
            "retrieval_time_ms": retrieval_time,
            "generation_time_ms": generation_time,
            "estimated_confidence": self._estimate_confidence(response, sources)
        }

    def _estimate_confidence(self, response: str, sources: List[SourceReference]) -> float:
        """
        Estimate confidence based on response characteristics and sources.

        Args:
            response: The response text
            sources: List of sources used

        Returns:
            Estimated confidence level (0-1)
        """
        # Base confidence on number of sources and response characteristics
        if not sources:
            return 0.0

        # If response indicates content not found, confidence is 0
        if ("not contained in the selected content" in response.lower() or
            "not present in the provided content" in response.lower()):
            return 0.0

        # Base confidence on number of sources (more sources = higher confidence, up to a point)
        source_confidence = min(1.0, len(sources) * 0.3)

        # Consider response length as a factor (very short responses might be less confident)
        length_factor = min(1.0, max(0.3, len(response) / 100))

        return source_confidence * length_factor

    async def format_response_for_ui(
        self,
        query_response: QueryResponse
    ) -> Dict[str, Any]:
        """
        Format a query response for UI consumption with proper attribution display.

        Args:
            query_response: The QueryResponse object to format

        Returns:
            Dictionary formatted for UI display
        """
        # Create a UI-friendly format
        ui_response = {
            "answer": query_response.response,
            "confidence": query_response.confidence,
            "status": query_response.status,
            "sources": [],
            "has_sources": len(query_response.sources) > 0,
            "sources_count": len(query_response.sources)
        }

        # Format sources for UI display
        for source in query_response.sources:
            ui_source = {
                "module": source.module,
                "chapter": source.chapter,
                "section": source.section,
                "heading_path": source.heading_path,
                "source_url": source.source_url,
                "preview": source.text_preview[:150] + "..." if len(source.text_preview) > 150 else source.text_preview,
                "display_text": f"{source.module} > {source.chapter} > {source.section}"
            }
            ui_response["sources"].append(ui_source)

        return ui_response

    async def merge_responses(
        self,
        responses: List[QueryResponse]
    ) -> QueryResponse:
        """
        Merge multiple query responses into a single response.

        Args:
            responses: List of QueryResponse objects to merge

        Returns:
            Merged QueryResponse object
        """
        if not responses:
            return QueryResponse(
                response="No responses to merge",
                sources=[],
                confidence=0.0,
                status="no_content"
            )

        # Combine response texts
        combined_response = "\n\n".join([r.response for r in responses if r.response])

        # Combine unique sources
        all_sources = []
        seen_source_ids = set()
        for response in responses:
            for source in response.sources:
                # Create a unique identifier for the source
                source_id = f"{source.module}_{source.chapter}_{source.section}_{source.source_url}"
                if source_id not in seen_source_ids:
                    all_sources.append(source)
                    seen_source_ids.add(source_id)

        # Calculate average confidence
        valid_confidences = [r.confidence for r in responses if r.confidence is not None]
        avg_confidence = sum(valid_confidences) / len(valid_confidences) if valid_confidences else 0.0

        # Determine overall status
        if any(r.status == "error" for r in responses):
            overall_status = "error"
        elif any(r.status == "no_content" for r in responses):
            overall_status = "no_content"
        else:
            overall_status = "success"

        return QueryResponse(
            response=combined_response,
            sources=all_sources,
            confidence=avg_confidence,
            status=overall_status
        )

    async def create_attribution_report(
        self,
        query: str,
        response: QueryResponse,
        original_search_results: List[ContentSearchResult]
    ) -> Dict[str, Any]:
        """
        Create a detailed attribution report for a response.

        Args:
            query: The original query
            response: The QueryResponse object
            original_search_results: Original search results used

        Returns:
            Dictionary with detailed attribution report
        """
        return {
            "query": query,
            "response_length": len(response.response),
            "sources_used_count": len(response.sources),
            "total_search_results": len(original_search_results),
            "sources_used_ratio": len(response.sources) / len(original_search_results) if original_search_results else 0,
            "source_urls_used": [s.source_url for s in response.sources],
            "modules_referenced": list(set(s.module for s in response.sources)),
            "confidence_level": response.confidence,
            "attribution_quality": self._assess_attribution_quality(response, original_search_results),
            "timestamp": datetime.utcnow().isoformat()
        }

    def _assess_attribution_quality(
        self,
        response: QueryResponse,
        search_results: List[ContentSearchResult]
    ) -> str:
        """
        Assess the quality of source attribution.

        Args:
            response: The QueryResponse object
            search_results: Original search results

        Returns:
            Quality assessment as a string ("high", "medium", "low")
        """
        if not search_results:
            return "low"  # No sources to attribute

        if not response.sources:
            return "low"  # No attribution made

        # Calculate ratio of sources used vs available
        ratio = len(response.sources) / len(search_results)

        # Consider confidence level as well
        if ratio >= 0.8 and response.confidence >= 0.7:
            return "high"
        elif ratio >= 0.5 and response.confidence >= 0.5:
            return "medium"
        else:
            return "low"

    async def health_check(self) -> bool:
        """
        Perform a health check of the response service.

        Returns:
            True if all dependencies are healthy, False otherwise
        """
        gen_healthy = await self.generation_service.health_check()
        retrieval_healthy = await self.retrieval_service.health_check()

        return gen_healthy and retrieval_healthy


# Global instance
response_service = ResponseService()