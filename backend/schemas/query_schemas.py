from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any
from datetime import datetime

from models.book_content import SourceReference


class QueryRequest(BaseModel):
    """
    Schema for query requests.
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The user's question")
    context_mode: str = Field(
        "global",
        regex=r"^(global|selection)$",
        description="Query context mode"
    )
    session_id: Optional[str] = Field(None, max_length=100, description="Session identifier")
    top_k: Optional[int] = Field(None, ge=1, le=20, description="Number of results to retrieve")

    @validator('query')
    def query_not_empty(cls, v):
        if not v or not v.strip():
            raise ValueError('Query cannot be empty or just whitespace')
        return v.strip()


class SelectionQueryRequest(BaseModel):
    """
    Schema for selection-based query requests.
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The question about selected text")
    selected_text: str = Field(..., min_length=1, max_length=5000, description="The user-selected text")
    context_mode: str = Field(
        "selection",
        regex=r"^(global|selection)$",
        description="Query context mode (must be 'selection' for this endpoint)"
    )
    session_id: Optional[str] = Field(None, max_length=100, description="Session identifier")

    @validator('query', 'selected_text')
    def text_not_empty(cls, v):
        if not v or not v.strip():
            raise ValueError('Text fields cannot be empty or just whitespace')
        return v.strip()


class QueryResponse(BaseModel):
    """
    Schema for query responses.
    """
    response: str = Field(..., description="The generated response to the query")
    sources: List[SourceReference] = Field(..., description="References to the book sections used in the response")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence level in the response")
    status: str = Field(..., regex=r"^(success|no_content|partial|error)$", description="Status of the query processing")
    processing_time_ms: Optional[float] = Field(None, description="Time taken to process the query in milliseconds")


class ContentFilterSchema(BaseModel):
    """
    Schema for content filtering.
    """
    module: Optional[str] = Field(None, max_length=100, description="Filter by module name")
    chapter: Optional[str] = Field(None, max_length=100, description="Filter by chapter name")
    section: Optional[str] = Field(None, max_length=100, description="Filter by section name")
    source_url: Optional[str] = Field(None, max_length=500, description="Filter by source URL")

    @validator('module', 'chapter', 'section', 'source_url')
    def validate_optional_strings(cls, v):
        if v is not None and (not v or not v.strip()):
            raise ValueError('Filter values cannot be empty or just whitespace')
        return v.strip() if v else v


class ChatHistoryResponse(BaseModel):
    """
    Schema for chat history responses.
    """
    session_id: str = Field(..., max_length=100, description="Session identifier")
    interactions: List[Dict[str, Any]] = Field(..., description="List of chat interactions")
    total_count: int = Field(..., ge=0, description="Total number of interactions in the session")
    timestamp: datetime = Field(..., description="Timestamp of the response")


class SourceAttribution(BaseModel):
    """
    Schema for source attribution in responses.
    """
    module: str = Field(..., min_length=1, max_length=100, description="Module name")
    chapter: str = Field(..., min_length=1, max_length=100, description="Chapter name")
    section: str = Field(..., min_length=1, max_length=100, description="Section name")
    heading_path: str = Field(..., min_length=1, max_length=500, description="Full heading hierarchy")
    source_url: str = Field(..., min_length=1, max_length=500, description="URL to the original content")
    text_preview: str = Field(..., max_length=500, description="Preview of the source text used")


class HealthCheckResponse(BaseModel):
    """
    Schema for health check responses.
    """
    status: str = Field(..., regex=r"^(healthy|unhealthy|ready|not ready|alive)$", description="Health status")
    service: str = Field(..., description="Name of the service being checked")
    timestamp: datetime = Field(..., description="Timestamp of the check")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional health details")


class ErrorResponse(BaseModel):
    """
    Schema for error responses.
    """
    detail: str = Field(..., description="Error message")
    error_code: Optional[str] = Field(None, description="Specific error code")
    timestamp: datetime = Field(..., description="Timestamp of the error")


class IngestionRequest(BaseModel):
    """
    Schema for content ingestion requests.
    """
    content: str = Field(..., min_length=10, max_length=50000, description="The content to be indexed")
    metadata: Dict[str, str] = Field(..., description="Metadata about the content")
    chunk_strategy: str = Field(
        "by_tokens",
        regex=r"^(by_tokens|by_headings)$",
        description="Strategy for chunking the content"
    )

    @validator('content')
    def content_not_too_short(cls, v):
        if len(v.strip()) < 10:
            raise ValueError('Content must be at least 10 characters long')
        return v


class IngestionResponse(BaseModel):
    """
    Schema for content ingestion responses.
    """
    status: str = Field(..., regex=r"^(success|partial|error)$", description="Status of the ingestion")
    indexed_chunks: int = Field(..., ge=0, description="Number of content chunks indexed")
    message: str = Field(..., description="Additional information about the indexing process")
    chunk_ids: List[str] = Field(..., description="IDs of the created chunks")
    processing_time_ms: Optional[float] = Field(None, description="Time taken to process the ingestion in milliseconds")


class ValidationRequest(BaseModel):
    """
    Schema for validation requests.
    """
    query: str = Field(..., min_length=1, max_length=1000, description="The original query")
    response: str = Field(..., min_length=1, max_length=10000, description="The generated response")
    context: str = Field(..., min_length=1, max_length=10000, description="The context used for generation")


class ValidationResponse(BaseModel):
    """
    Schema for validation responses.
    """
    is_valid: bool = Field(..., description="Whether the response is properly grounded in the context")
    validation_message: str = Field(..., description="Message explaining the validation result")
    confidence_score: Optional[float] = Field(None, ge=0.0, le=1.0, description="Confidence in the validation")