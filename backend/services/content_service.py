from typing import List, Dict, Any, Optional
import asyncio
from datetime import datetime

from clients.qdrant_client import qdrant_service
from clients.cohere_client import cohere_service
from clients.postgres_client import postgres_service, BookContent as BookContentDB
from utils.chunking_utils import chunking_utils, ContentChunk
from utils.parsing_utils import parsing_utils, ParsedContent
from utils.embedding_utils import embedding_utils
from models.book_content import BookContentMetadata, BookContentIngestRequest, BookContentIngestResponse, ContentSearchResult


class ContentService:
    """Service class for handling book content ingestion and retrieval."""

    def __init__(self):
        """Initialize the content service with required clients."""
        self.qdrant_service = qdrant_service
        self.cohere_service = cohere_service
        self.postgres_service = postgres_service

    async def ingest_content(
        self,
        content: str,
        metadata: BookContentMetadata,
        chunk_strategy: str = "by_tokens"
    ) -> BookContentIngestResponse:
        """
        Ingest book content by chunking, embedding, and storing in Qdrant.

        Args:
            content: The content to ingest
            metadata: Metadata about the content
            chunk_strategy: Strategy for chunking ("by_tokens", "by_headings")

        Returns:
            BookContentIngestResponse with ingestion results
        """
        try:
            # Clean the content
            cleaned_content = parsing_utils.clean_content(content)

            # Chunk the content based on the strategy
            if chunk_strategy == "by_headings":
                chunks = chunking_utils.chunk_by_headings(
                    cleaned_content,
                    max_tokens=512,
                    overlap_tokens=128
                )
            else:  # default to by_tokens
                chunks = chunking_utils.chunk_by_tokens(
                    cleaned_content,
                    max_tokens=512,
                    overlap_tokens=128
                )

            if not chunks:
                return BookContentIngestResponse(
                    status="error",
                    indexed_chunks=0,
                    message="No content chunks were created",
                    chunk_ids=[]
                )

            # Prepare content and metadata for embedding
            texts_to_embed = [chunk.text for chunk in chunks]
            metadata_list = []

            for chunk in chunks:
                chunk_metadata = {
                    "module": metadata.module,
                    "chapter": metadata.chapter,
                    "section": metadata.section,
                    "heading_path": metadata.heading_path or chunk.heading_context,
                    "source_url": metadata.source_url,
                    "created_at": metadata.created_at.isoformat() if metadata.created_at else datetime.utcnow().isoformat()
                }
                metadata_list.append(chunk_metadata)

            # Generate embeddings
            embeddings = await embedding_utils.generate_embeddings(texts_to_embed)

            # Store in Qdrant
            chunk_ids = await self.qdrant_service.store_embeddings(
                contents=texts_to_embed,
                embeddings=embeddings,
                metadata_list=metadata_list
            )

            # Store metadata in Postgres (not the embeddings, just for reference)
            for chunk_id, chunk in zip(chunk_ids, chunks):
                # Create a record in Postgres for the metadata
                self.postgres_service.SessionLocal().add(
                    BookContentDB(
                        content_id=chunk_id,
                        module=metadata.module,
                        chapter=metadata.chapter,
                        section=metadata.section,
                        heading_path=metadata.heading_path or chunk.heading_context,
                        source_url=metadata.source_url,
                        content_preview=chunk.text[:200]  # Store a preview
                    )
                )
                self.postgres_service.SessionLocal().commit()

            return BookContentIngestResponse(
                status="success",
                indexed_chunks=len(chunk_ids),
                message=f"Content successfully indexed with {len(chunk_ids)} chunks",
                chunk_ids=chunk_ids
            )

        except Exception as e:
            return BookContentIngestResponse(
                status="error",
                indexed_chunks=0,
                message=f"Error during content ingestion: {str(e)}",
                chunk_ids=[]
            )

    async def ingest_parsed_content(
        self,
        parsed_content: ParsedContent,
        source_url: str,
        chunk_strategy: str = "by_tokens"
    ) -> BookContentIngestResponse:
        """
        Ingest parsed content with automatic metadata extraction.

        Args:
            parsed_content: Parsed content with metadata
            source_url: URL of the source content
            chunk_strategy: Strategy for chunking

        Returns:
            BookContentIngestResponse with ingestion results
        """
        # Extract module, chapter, section from source URL
        module, chapter, section, heading_path = parsing_utils.extract_module_chapter_section_from_path(source_url)

        # Create metadata
        metadata = BookContentMetadata(
            id="",  # Will be generated during storage
            content=parsed_content.content,
            module=parsed_content.metadata.get('module', module),
            chapter=parsed_content.metadata.get('chapter', chapter),
            section=parsed_content.metadata.get('section', section),
            heading_path=parsed_content.metadata.get('heading_path', heading_path),
            source_url=source_url,
            created_at=datetime.utcnow()
        )

        return await self.ingest_content(
            content=parsed_content.content,
            metadata=metadata,
            chunk_strategy=chunk_strategy
        )

    async def search_content(
        self,
        query: str,
        top_k: int = 5,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[ContentSearchResult]:
        """
        Search for content similar to the query.

        Args:
            query: The query to search for
            top_k: Number of results to return
            filters: Optional filters to apply

        Returns:
            List of ContentSearchResult objects
        """
        # Generate embedding for the query
        query_embedding = await embedding_utils.generate_single_embedding(query)

        # Search in Qdrant
        search_results = await self.qdrant_service.search_similar(
            query_embedding=query_embedding,
            top_k=top_k,
            filters=filters
        )

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

    async def get_content_by_source_url(self, source_url: str) -> List[Dict[str, Any]]:
        """
        Retrieve all content chunks for a specific source URL.

        Args:
            source_url: URL of the source content

        Returns:
            List of content chunks with metadata
        """
        # This would require a custom method in Qdrant service to retrieve by source_url
        # For now, we'll return empty list as this functionality would need to be implemented
        # in the Qdrant service with a scroll or search operation
        pass

    async def delete_content_by_source_url(self, source_url: str) -> bool:
        """
        Delete all content chunks for a specific source URL.

        Args:
            source_url: URL of the source content to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            await self.qdrant_service.delete_by_source_url(source_url)
            return True
        except Exception:
            return False

    async def update_content(
        self,
        source_url: str,
        new_content: str,
        metadata: BookContentMetadata
    ) -> BookContentIngestResponse:
        """
        Update existing content by deleting old chunks and ingesting new ones.

        Args:
            source_url: URL of the content to update
            new_content: New content to ingest
            metadata: Updated metadata

        Returns:
            BookContentIngestResponse with update results
        """
        # First, delete existing content
        delete_success = await self.delete_content_by_source_url(source_url)

        if not delete_success:
            return BookContentIngestResponse(
                status="error",
                indexed_chunks=0,
                message="Failed to delete existing content before update",
                chunk_ids=[]
            )

        # Then ingest the new content
        return await self.ingest_content(new_content, metadata)

    async def validate_ingestion(
        self,
        original_content: str,
        source_url: str
    ) -> tuple[bool, str]:
        """
        Validate that content was properly ingested by checking for similarity.

        Args:
            original_content: The original content that was ingested
            source_url: URL of the ingested content

        Returns:
            Tuple of (is_valid, validation_message)
        """
        # This would require searching for content similar to the original
        # and checking if we get reasonable matches
        # For now, we'll return a basic validation
        search_results = await self.search_content(original_content, top_k=1)

        if search_results:
            best_match = search_results[0]
            # Simple check: if the best match has a reasonable similarity score
            if best_match.score > 0.7:
                return True, f"Content validated with best match score: {best_match.score}"
            else:
                return False, f"Content may not be properly indexed, best match score: {best_match.score}"
        else:
            return False, "No content found for validation"

    async def get_all_source_urls(self) -> List[str]:
        """
        Get all source URLs that have been ingested.

        Returns:
            List of source URLs
        """
        return await self.qdrant_service.get_all_source_urls()

    async def health_check(self) -> bool:
        """
        Perform a health check of the content service.

        Returns:
            True if all dependencies are healthy, False otherwise
        """
        qdrant_healthy = await self.qdrant_service.health_check()
        cohere_healthy = await self.cohere_service.health_check()
        postgres_healthy = self.postgres_service.health_check()

        return qdrant_healthy and cohere_healthy and postgres_healthy


# Global instance
content_service = ContentService()