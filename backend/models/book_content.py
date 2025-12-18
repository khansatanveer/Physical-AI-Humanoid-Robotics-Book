from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class SourceReference(BaseModel):
    """
    Reference to a source in the book content.
    Used for attribution in responses.
    """
    module: str
    chapter: str
    section: str
    heading_path: str
    source_url: str
    text_preview: str


class BookContentMetadata(BaseModel):
    """
    Metadata for book content chunks.
    This represents the structure of content stored in the vector database.
    """
    id: str  # ID from Qdrant
    content: str  # The actual text content of the chunk
    module: str  # The module name this content belongs to
    chapter: str  # The chapter name within the module
    section: str  # The section name within the chapter
    heading_path: str  # Full heading hierarchy (e.g., "Module 1 > Chapter 2 > Section 3")
    source_url: str  # URL/path to the original content location
    created_at: Optional[datetime] = None  # When this content was indexed
    updated_at: Optional[datetime] = None  # When this content was last updated


class BookContentIngestRequest(BaseModel):
    """
    Request model for ingesting book content.
    """
    content: str  # The content to be indexed
    metadata: BookContentMetadata  # Metadata about the content


class BookContentIngestResponse(BaseModel):
    """
    Response model for content ingestion.
    """
    status: str  # success, partial, error
    indexed_chunks: int  # Number of content chunks indexed
    message: str  # Additional information about the indexing process
    chunk_ids: List[str]  # IDs of the created chunks


class ContentSearchResult(BaseModel):
    """
    Result model for content search operations.
    """
    id: str  # ID from Qdrant
    content: str  # The content text
    module: str
    chapter: str
    section: str
    heading_path: str
    source_url: str
    score: float  # Similarity score


class ContentFilter(BaseModel):
    """
    Filter model for content search operations.
    """
    module: Optional[str] = None
    chapter: Optional[str] = None
    section: Optional[str] = None
    source_url: Optional[str] = None