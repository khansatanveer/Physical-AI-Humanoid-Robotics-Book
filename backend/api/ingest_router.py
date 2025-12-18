from fastapi import APIRouter, HTTPException, status
from typing import Dict, Any
import logging
from models.book_content import BookContentIngestRequest
from services.content_service import content_service
from exceptions import RagHTTPException, handle_validation_error, handle_content_not_found_error, handle_generation_error
from models.chat_interaction import StandardResponse

router = APIRouter(tags=["ingest"])

logger = logging.getLogger(__name__)


@router.post("/ingest", response_model=StandardResponse)
async def ingest_content(request: BookContentIngestRequest) -> StandardResponse:
    """
    Ingest book content into the vector database.

    This endpoint processes and indexes book content with associated metadata
    for retrieval during querying.
    """
    try:
        # Validate the request
        if not request.content or not request.content.strip():
            raise handle_validation_error("Content cannot be empty")

        if not request.metadata:
            raise handle_validation_error("Metadata is required")

        # Process the ingestion
        result = await content_service.ingest_content(
            content=request.content,
            metadata=request.metadata
        )

        if result.status == "error":
            raise handle_content_not_found_error()

        return StandardResponse(
            response=result.message,
            sources=[],
            confidence=1.0,
            status="success",
            additional_info={
                "indexed_chunks": result.indexed_chunks,
                "chunk_ids": result.chunk_ids
            }
        )

    except RagHTTPException:
        raise
    except Exception as e:
        logger.error(f"Error during content ingestion: {str(e)}", exc_info=True)
        raise handle_generation_error(f"Error ingesting content: {str(e)}")


@router.get("/ingest/health", response_model=Dict[str, Any])
async def ingest_health_check() -> Dict[str, Any]:
    """
    Health check endpoint for the ingestion service.
    """
    try:
        is_healthy = await content_service.health_check()
        return {
            "status": "healthy" if is_healthy else "unhealthy",
            "service": "ingest_service",
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "status": "unhealthy",
            "service": "ingest_service",
            "timestamp": __import__('datetime').datetime.now().isoformat(),
            "error": str(e)
        }