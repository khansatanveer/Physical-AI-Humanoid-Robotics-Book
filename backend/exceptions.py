from typing import Optional
from fastapi import HTTPException, status
from pydantic import BaseModel


class RAGException(Exception):
    """Base exception class for RAG-related errors."""

    def __init__(self, message: str, error_code: Optional[str] = None, details: Optional[dict] = None):
        self.message = message
        self.error_code = error_code or "RAG_ERROR"
        self.details = details or {}
        super().__init__(self.message)

    def __str__(self):
        return f"{self.error_code}: {self.message}"


class ContentNotInSelectedTextError(RAGException):
    """Raised when the answer is not contained in the selected text."""

    def __init__(self, message: str = "The answer is not contained in the selected content."):
        super().__init__(
            message=message,
            error_code="CONTENT_NOT_IN_SELECTED",
            details={"reason": "content_not_found_in_selection"}
        )


class ContentNotInProvidedContentError(RAGException):
    """Raised when the requested information is not present in the provided content."""

    def __init__(self, message: str = "The requested information is not present in the provided content."):
        super().__init__(
            message=message,
            error_code="CONTENT_NOT_IN_PROVIDED",
            details={"reason": "content_not_found_in_provided_context"}
        )


class InsufficientContentError(RAGException):
    """Raised when there is insufficient content to answer a query."""

    def __init__(self, message: str = "Insufficient content to answer the query."):
        super().__init__(
            message=message,
            error_code="INSUFFICIENT_CONTENT",
            details={"reason": "insufficient_content_for_query"}
        )


class EmbeddingGenerationError(RAGException):
    """Raised when there's an error generating embeddings."""

    def __init__(self, message: str = "Error generating embeddings."):
        super().__init__(
            message=message,
            error_code="EMBEDDING_GENERATION_ERROR",
            details={"reason": "embedding_generation_failed"}
        )


class VectorSearchError(RAGException):
    """Raised when there's an error performing vector search."""

    def __init__(self, message: str = "Error performing vector search."):
        super().__init__(
            message=message,
            error_code="VECTOR_SEARCH_ERROR",
            details={"reason": "vector_search_failed"}
        )


class ContentRetrievalError(RAGException):
    """Raised when there's an error retrieving content."""

    def __init__(self, message: str = "Error retrieving content."):
        super().__init__(
            message=message,
            error_code="CONTENT_RETRIEVAL_ERROR",
            details={"reason": "content_retrieval_failed"}
        )


class GenerationError(RAGException):
    """Raised when there's an error during text generation."""

    def __init__(self, message: str = "Error during text generation."):
        super().__init__(
            message=message,
            error_code="GENERATION_ERROR",
            details={"reason": "text_generation_failed"}
        )


class ConfigurationError(RAGException):
    """Raised when there's a configuration error."""

    def __init__(self, message: str = "Configuration error."):
        super().__init__(
            message=message,
            error_code="CONFIGURATION_ERROR",
            details={"reason": "configuration_issue"}
        )


class ValidationError(RAGException):
    """Raised when there's a validation error."""

    def __init__(self, message: str = "Validation error."):
        super().__init__(
            message=message,
            error_code="VALIDATION_ERROR",
            details={"reason": "validation_failed"}
        )


class ExternalServiceError(RAGException):
    """Raised when there's an error with an external service (Cohere, Qdrant, etc.)."""

    def __init__(self, service_name: str, message: str = "External service error."):
        super().__init__(
            message=f"{service_name} service error: {message}",
            error_code="EXTERNAL_SERVICE_ERROR",
            details={"service": service_name, "reason": "external_service_unavailable"}
        )


class RagHTTPException(HTTPException):
    """Custom HTTP exception for RAG-specific errors."""

    def __init__(
        self,
        status_code: int,
        detail: str,
        headers: Optional[dict] = None,
        error_code: Optional[str] = None
    ):
        super().__init__(status_code=status_code, detail=detail, headers=headers)
        self.error_code = error_code


def handle_content_not_found_error(query_type: str = "global") -> RagHTTPException:
    """Create an appropriate HTTP exception for content not found scenarios."""
    if query_type == "selection":
        error = ContentNotInSelectedTextError()
        return RagHTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=error.message,
            error_code=error.error_code
        )
    else:
        error = ContentNotInProvidedContentError()
        return RagHTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=error.message,
            error_code=error.error_code
        )


def handle_insufficient_content_error() -> RagHTTPException:
    """Create an HTTP exception for insufficient content scenarios."""
    error = InsufficientContentError()
    return RagHTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=error.message,
        error_code=error.error_code
    )


def handle_generation_error(message: str = "Error generating response") -> RagHTTPException:
    """Create an HTTP exception for generation errors."""
    error = GenerationError(message)
    return RagHTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=error.message,
        error_code=error.error_code
    )


def handle_embedding_error(message: str = "Error generating embeddings") -> RagHTTPException:
    """Create an HTTP exception for embedding errors."""
    error = EmbeddingGenerationError(message)
    return RagHTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=error.message,
        error_code=error.error_code
    )


def handle_search_error(message: str = "Error searching for content") -> RagHTTPException:
    """Create an HTTP exception for search errors."""
    error = VectorSearchError(message)
    return RagHTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=error.message,
        error_code=error.error_code
    )


def handle_validation_error(message: str = "Validation error") -> RagHTTPException:
    """Create an HTTP exception for validation errors."""
    error = ValidationError(message)
    return RagHTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=error.message,
        error_code=error.error_code
    )


def handle_external_service_error(service_name: str, message: str = "Service unavailable") -> RagHTTPException:
    """Create an HTTP exception for external service errors."""
    error = ExternalServiceError(service_name, message)
    return RagHTTPException(
        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
        detail=error.message,
        error_code=error.error_code
    )


# Error response model for API responses
class ErrorResponse(BaseModel):
    """Model for error responses in the API."""
    detail: str
    error_code: Optional[str] = None
    timestamp: str
    path: Optional[str] = None
    request_id: Optional[str] = None