from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class QueryLogRequest(BaseModel):
    """
    Request model for query logging operations.
    """
    query: str  # The original query
    response_status: str  # "success", "error", "no_content"
    retrieval_time: Optional[float] = None  # Time taken for content retrieval (ms)
    generation_time: Optional[float] = None  # Time taken for response generation (ms)
    total_time: Optional[float] = None  # Total processing time (ms)
    retrieved_count: Optional[int] = None  # Number of content chunks retrieved
    user_agent: Optional[str] = None  # Client information (optional)
    ip_address: Optional[str] = None  # User IP for analytics (respecting privacy)


class QueryLog(BaseModel):
    """
    Model representing a query log entry.
    """
    id: str  # Unique identifier for the log entry
    query: str  # The original query
    response_status: str  # Status of the response ("success", "error", "no_content")
    retrieval_time: Optional[float] = None  # Time taken for content retrieval (ms)
    generation_time: Optional[float] = None  # Time taken for response generation (ms)
    total_time: Optional[float] = None  # Total processing time (ms)
    retrieved_count: Optional[int] = None  # Number of content chunks retrieved
    user_agent: Optional[str] = None  # Client information (optional)
    ip_address: Optional[str] = None  # User IP for analytics (respecting privacy)
    timestamp: datetime  # When the query was processed


class QueryLogResponse(BaseModel):
    """
    Response model for query log operations.
    """
    logs: list[QueryLog]  # List of query logs
    total_count: int  # Total number of logs returned
    page: int  # Current page number (for pagination)
    per_page: int  # Number of logs per page (for pagination)


class QueryPerformanceMetrics(BaseModel):
    """
    Model for query performance metrics.
    """
    avg_retrieval_time: float  # Average retrieval time in ms
    avg_generation_time: float  # Average generation time in ms
    avg_total_time: float  # Average total processing time in ms
    success_rate: float  # Percentage of successful queries (0-1)
    top_queries: list[str]  # Most common queries
    error_rate: float  # Percentage of failed queries (0-1)
    time_period: str  # Time period for the metrics (e.g., "last_hour", "last_day")