from typing import List, Dict, Any, Optional
import asyncio
from datetime import datetime

from clients.cohere_client import cohere_service
from clients.qdrant_client import qdrant_service
from clients.postgres_client import postgres_service
from utils.embedding_utils import embedding_utils
from utils.generation_utils import generation_utils, GenerationMode
from models.book_content import ContentSearchResult, SourceReference
from models.chat_interaction import ChatInteraction, ChatInteractionRequest, QueryResponse


class QueryService:
    """Service class for handling query operations."""

    def __init__(self):
        """Initialize the query service with required clients."""
        self.cohere_service = cohere_service
        self.qdrant_service = qdrant_service
        self.postgres_service = postgres_service
        self.generation_utils = generation_utils

    async def process_global_query(
        self,
        query: str,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None,
        session_id: Optional[str] = None
    ) -> QueryResponse:
        """
        Process a global query against all indexed content.

        Args:
            query: The user's query
            top_k: Number of results to retrieve
            filters: Optional filters to apply
            session_id: Optional session identifier

        Returns:
            QueryResponse with the answer and sources
        """
        start_time = datetime.utcnow()

        try:
            # Generate embedding for the query
            query_embedding = await embedding_utils.generate_single_embedding(query)

            # Search in Qdrant
            search_results = await self.qdrant_service.search_similar(
                query_embedding=query_embedding,
                top_k=top_k,
                filters=filters
            )

            if not search_results:
                # No relevant content found
                return QueryResponse(
                    response="The requested information is not present in the provided content.",
                    sources=[],
                    confidence=0.0,
                    status="no_content"
                )

            # Prepare context from search results
            context_chunks = []
            sources = []

            for result in search_results:
                context_chunks.append({
                    "content": result["content"],
                    "module": result["module"],
                    "chapter": result["chapter"],
                    "section": result["section"],
                    "heading_path": result["heading_path"],
                    "source_url": result["source_url"]
                })

                source = SourceReference(
                    module=result["module"],
                    chapter=result["chapter"],
                    section=result["section"],
                    heading_path=result["heading_path"],
                    source_url=result["source_url"],
                    text_preview=result["content"][:200] + "..." if len(result["content"]) > 200 else result["content"]
                )
                sources.append(source)

            # Generate response using the context
            response, generated_sources = await generation_utils.generate_answer_with_sources(
                query=query,
                context_chunks=context_chunks,
                mode=GenerationMode.GROUNDED,
                max_tokens=500,
                temperature=0.3
            )

            # Calculate confidence based on the highest similarity score
            confidence = max([result["score"] for result in search_results]) if search_results else 0.0

            # Log the query for analytics
            retrieval_time = (datetime.utcnow() - start_time).total_seconds() * 1000  # in ms
            await self._log_query(
                query=query,
                response_status="success",
                retrieval_time=retrieval_time,
                retrieved_count=len(search_results)
            )

            return QueryResponse(
                response=response,
                sources=sources,
                confidence=confidence,
                status="success"
            )

        except Exception as e:
            # Log the error
            await self._log_query(
                query=query,
                response_status="error",
                retrieval_time=(datetime.utcnow() - start_time).total_seconds() * 1000
            )

            return QueryResponse(
                response="An error occurred while processing your query. Please try again.",
                sources=[],
                confidence=0.0,
                status="error"
            )

    async def process_selection_query(
        self,
        query: str,
        selected_text: str,
        session_id: Optional[str] = None
    ) -> QueryResponse:
        """
        Process a query using only the selected text as context.

        Args:
            query: The user's query about the selected text
            selected_text: The user-selected text to use as context
            session_id: Optional session identifier

        Returns:
            QueryResponse with the answer and sources
        """
        start_time = datetime.utcnow()

        try:
            # Validate that selected text is provided
            if not selected_text or not selected_text.strip():
                return QueryResponse(
                    response="The answer is not contained in the selected content.",
                    sources=[],
                    confidence=0.0,
                    status="no_content"
                )

            # Prepare context from selected text
            context_chunks = [{
                "content": selected_text,
                "module": "Selected Text",
                "chapter": "User Selection",
                "section": "Selected Content",
                "heading_path": "User > Selection > Context",
                "source_url": "user_selection"
            }]

            # Generate response using only the selected text as context
            response, sources = await generation_utils.generate_answer_with_sources(
                query=query,
                context_chunks=context_chunks,
                mode=GenerationMode.SELECTION_ONLY,
                max_tokens=500,
                temperature=0.3
            )

            # Calculate confidence (for selection mode, we use a fixed confidence or derive from content length/quality)
            confidence = min(1.0, len(selected_text) / 1000)  # Simple heuristic

            # Log the query for analytics
            retrieval_time = (datetime.utcnow() - start_time).total_seconds() * 1000  # in ms
            await self._log_query(
                query=query,
                response_status="success",
                retrieval_time=retrieval_time,
                retrieved_count=1  # We used 1 chunk of selected text
            )

            return QueryResponse(
                response=response,
                sources=sources,
                confidence=confidence,
                status="success"
            )

        except Exception as e:
            # Log the error
            await self._log_query(
                query=query,
                response_status="error",
                retrieval_time=(datetime.utcnow() - start_time).total_seconds() * 1000
            )

            return QueryResponse(
                response="An error occurred while processing your query. Please try again.",
                sources=[],
                confidence=0.0,
                status="error"
            )

    async def _log_query(
        self,
        query: str,
        response_status: str,
        retrieval_time: Optional[float] = None,
        generation_time: Optional[float] = None,
        retrieved_count: Optional[int] = None
    ):
        """
        Log query information for analytics.

        Args:
            query: The original query
            response_status: Status of the response
            retrieval_time: Time taken for retrieval
            generation_time: Time taken for generation
            retrieved_count: Number of items retrieved
        """
        try:
            self.postgres_service.save_query_log(
                query=query,
                response_status=response_status,
                retrieval_time=retrieval_time,
                generation_time=generation_time,
                total_time=retrieval_time,
                retrieved_count=retrieved_count
            )
        except Exception:
            # If logging fails, don't affect the main query processing
            pass

    async def save_chat_interaction(
        self,
        user_id: Optional[str],
        session_id: Optional[str],
        query: str,
        response: str,
        query_type: str,
        selected_text: Optional[str] = None,
        sources: Optional[List[SourceReference]] = None
    ) -> str:
        """
        Save a chat interaction to the database.

        Args:
            user_id: Optional user identifier
            session_id: Optional session identifier
            query: The user's query
            response: The system's response
            query_type: Type of query ("global" or "selection")
            selected_text: Selected text (for selection queries)
            sources: List of sources used

        Returns:
            ID of the saved interaction
        """
        try:
            sources_json = None
            if sources:
                # Convert sources to JSON string for storage
                import json
                sources_json = json.dumps([source.dict() for source in sources])

            interaction_id = self.postgres_service.save_chat_interaction(
                user_id=user_id,
                session_id=session_id,
                query=query,
                response=response,
                query_type=query_type,
                selected_text=selected_text,
                source_attribution=sources_json
            )

            return interaction_id
        except Exception:
            # If saving fails, return None but don't affect the main response
            return ""

    async def validate_query_response(
        self,
        query: str,
        response: str,
        context: str
    ) -> tuple[bool, str]:
        """
        Validate that the response is properly grounded in the context.

        Args:
            query: The original query
            response: The generated response
            context: The context used to generate the response

        Returns:
            Tuple of (is_valid, validation_message)
        """
        return await generation_utils.validate_answer_grounding(
            answer=response,
            source_context=context,
            query=query
        )

    async def get_conversation_history(
        self,
        session_id: str
    ) -> List[ChatInteraction]:
        """
        Get the conversation history for a session.

        Args:
            session_id: Session identifier

        Returns:
            List of chat interactions in the session
        """
        try:
            interactions = self.postgres_service.get_chat_interactions_by_session(session_id)
            # Convert database models to Pydantic models if needed
            # For now, we'll return the DB models directly
            return interactions
        except Exception:
            return []

    async def health_check(self) -> bool:
        """
        Perform a health check of the query service.

        Returns:
            True if all dependencies are healthy, False otherwise
        """
        cohere_healthy = await self.cohere_service.health_check()
        qdrant_healthy = await self.qdrant_service.health_check()
        postgres_healthy = self.postgres_service.health_check()

        return cohere_healthy and qdrant_healthy and postgres_healthy


# Global instance
query_service = QueryService()