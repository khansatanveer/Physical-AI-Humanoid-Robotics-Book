from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

from .book_content import SourceReference


class ChatInteractionRequest(BaseModel):
    """
    Request model for chat interactions.
    """
    query: str  # The user's question
    context_mode: str = "global"  # "global" or "selection"
    selected_text: Optional[str] = None  # Text selected by user (for selection queries only)
    session_id: Optional[str] = None  # Session identifier for conversation history


class FilteredQueryRequest(BaseModel):
    """
    Request model for filtered queries.
    """
    query: str  # The user's question
    context_mode: str = "global"  # Always "global" for filtered queries
    filters: Optional[dict] = None  # Filters for content search
    session_id: Optional[str] = None  # Session identifier for conversation history
    top_k: Optional[int] = None  # Number of results to retrieve


class SelectionQueryRequest(BaseModel):
    """
    Request model specifically for selection-based queries.
    """
    query: str  # The question about the selected text
    selected_text: str  # The user-selected text to query against
    context_mode: str = "selection"  # Always "selection" for this endpoint
    session_id: Optional[str] = None  # Session identifier for conversation history


class QueryResponse(BaseModel):
    """
    Response model for query operations.
    """
    response: str  # The generated response to the query
    sources: List[SourceReference]  # References to the book sections used in the response
    confidence: float  # Confidence level in the response (0-1)
    status: str  # Status of the query processing ("success", "no_content", "partial")


class ChatInteraction(BaseModel):
    """
    Model representing a chat interaction.
    """
    id: str  # Unique identifier for the interaction
    user_id: Optional[str]  # Identifier for the user (optional for anonymous)
    session_id: Optional[str]  # Session identifier for conversation history
    query: str  # The user's original question
    response: str  # The system's generated response
    query_type: str  # Type of query ("global" or "selection")
    selected_text: Optional[str] = None  # Text selected by user (for selection queries only)
    retrieved_chunks: Optional[List[str]] = None  # IDs of content chunks used to generate response
    confidence_score: Optional[float] = None  # Confidence level in the response (0-1)
    source_attribution: Optional[List[SourceReference]] = None  # Source references for the response
    timestamp: datetime  # When the interaction occurred


class ChatHistoryResponse(BaseModel):
    """
    Response model for chat history operations.
    """
    interactions: List[ChatInteraction]  # List of chat interactions in the session
    session_id: str  # Session identifier
    total_count: int  # Total number of interactions in the session


class StandardResponse(BaseModel):
    """
    Standard response model for all API responses.
    """
    response: str  # The generated response
    sources: List[SourceReference]  # References to the book sections used in the response
    confidence: float  # Confidence level in the response (0-1)
    status: str  # Status of the operation ("success", "no_content", "error")
    additional_info: Optional[dict] = None  # Additional information specific to the operation