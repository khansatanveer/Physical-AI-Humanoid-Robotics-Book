from fastapi import APIRouter, HTTPException, Depends, Request
from typing import Optional
import time

from models.chat_interaction import ChatInteractionRequest, SelectionQueryRequest, QueryResponse, FilteredQueryRequest
from models.book_content import ContentFilter
from services.query_service import query_service
from services.content_service import content_service
from config import settings


router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(
    request: ChatInteractionRequest,
    req: Request  # For logging IP address and user agent
):
    """
    Handle global content queries using RAG.

    Accepts a user query and returns a response based on indexed book content.
    """
    start_time = time.time()

    # Validate the request
    if not request.query or not request.query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    if request.context_mode not in ["global", "selection"]:
        raise HTTPException(status_code=400, detail="context_mode must be 'global' or 'selection'")

    # For global queries, context_mode should be "global"
    if request.context_mode != "global":
        raise HTTPException(status_code=400, detail="This endpoint is for global queries only")

    try:
        # Process the query using the query service
        response = await query_service.process_global_query(
            query=request.query,
            top_k=settings.top_k,
            session_id=request.session_id
        )

        # Calculate processing time
        processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Log the interaction if successful
        if response.status == "success":
            interaction_id = await query_service.save_chat_interaction(
                user_id=None,  # Could extract from auth if available
                session_id=request.session_id,
                query=request.query,
                response=response.response,
                query_type="global",
                sources=response.sources
            )

        return response

    except Exception as e:
        # Log the error
        await query_service._log_query(
            query=request.query,
            response_status="error",
            retrieval_time=(time.time() - start_time) * 1000
        )

        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.post("/query/selection", response_model=QueryResponse)
async def selection_query_endpoint(
    request: SelectionQueryRequest,
    req: Request  # For logging IP address and user agent
):
    """
    Handle queries using only selected text as context.

    Accepts a user query and selected text, returns a response based only on the selected text.
    """
    start_time = time.time()

    # Validate the request
    if not request.query or not request.query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    if not request.selected_text or not request.selected_text.strip():
        raise HTTPException(status_code=400, detail="Selected text cannot be empty")

    # For selection queries, context_mode should be "selection"
    if request.context_mode != "selection":
        raise HTTPException(status_code=400, detail="This endpoint is for selection queries only")

    try:
        # Process the selection query using the query service
        response = await query_service.process_selection_query(
            query=request.query,
            selected_text=request.selected_text,
            session_id=request.session_id
        )

        # Calculate processing time
        processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds

        # Log the interaction if successful
        if response.status == "success":
            interaction_id = await query_service.save_chat_interaction(
                user_id=None,  # Could extract from auth if available
                session_id=request.session_id,
                query=request.query,
                response=response.response,
                query_type="selection",
                selected_text=request.selected_text,
                sources=response.sources
            )

        return response

    except Exception as e:
        # Log the error
        await query_service._log_query(
            query=request.query,
            response_status="error",
            retrieval_time=(time.time() - start_time) * 1000
        )

        raise HTTPException(status_code=500, detail=f"Error processing selection query: {str(e)}")


@router.post("/query/filtered", response_model=QueryResponse)
async def filtered_query_endpoint(
    request: FilteredQueryRequest
):
    """
    Handle global content queries with filters.

    Accepts a user query and optional filters, returns a response based on indexed book content.
    """
    start_time = time.time()

    # Validate the request
    if not request.query or not request.query.strip():
        raise HTTPException(status_code=400, detail="Query cannot be empty")

    if request.context_mode != "global":
        raise HTTPException(status_code=400, detail="This endpoint is for global queries only")

    try:
        # Process the query using the query service with filters
        response = await query_service.process_global_query(
            query=request.query,
            top_k=request.top_k or settings.top_k,
            filters=request.filters,
            session_id=request.session_id
        )

        return response

    except Exception as e:
        # Log the error
        await query_service._log_query(
            query=request.query,
            response_status="error",
            retrieval_time=(time.time() - start_time) * 1000
        )

        raise HTTPException(status_code=500, detail=f"Error processing filtered query: {str(e)}")


@router.get("/query/history/{session_id}")
async def get_conversation_history(session_id: str):
    """
    Retrieve conversation history for a session.

    Returns the chat history for a given session ID.
    """
    try:
        history = await query_service.get_conversation_history(session_id)
        return {"session_id": session_id, "history": history, "count": len(history)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving history: {str(e)}")


@router.get("/query/sources")
async def get_all_source_urls():
    """
    Get all source URLs that have been ingested.

    Returns a list of all source URLs in the system.
    """
    try:
        source_urls = await content_service.get_all_source_urls()
        return {"source_urls": source_urls, "count": len(source_urls)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving source URLs: {str(e)}")


@router.get("/query/health")
async def query_health_check():
    """
    Health check for the query service.

    Returns the health status of the query service and its dependencies.
    """
    try:
        is_healthy = await query_service.health_check()
        return {
            "status": "healthy" if is_healthy else "unhealthy",
            "service": "query_service",
            "timestamp": __import__('datetime').datetime.utcnow().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")