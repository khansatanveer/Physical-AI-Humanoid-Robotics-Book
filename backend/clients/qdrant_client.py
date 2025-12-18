from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import uuid

from config import settings


class QdrantService:
    """Service class to handle Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client with configuration."""
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,  # Using HTTP for better compatibility
        )
        self.collection_name = settings.qdrant_collection_name

    async def initialize_collection(self):
        """Initialize the collection if it doesn't exist."""
        try:
            collections = self.client.get_collections()
            collection_exists = any(
                collection.name == self.collection_name
                for collection in collections.collections
            )

            if not collection_exists:
                # Create collection with vector size of 1024 (for Cohere embed-multilingual-v3.0)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
                )

                # Create payload index for faster filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="module",
                    field_schema=models.PayloadSchemaType.KEYWORD,
                )
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="chapter",
                    field_schema=models.PayloadSchemaType.KEYWORD,
                )
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section",
                    field_schema=models.PayloadSchemaType.KEYWORD,
                )
        except Exception as e:
            print(f"Error initializing collection: {e}")
            raise

    async def store_embedding(
        self,
        content: str,
        embedding: List[float],
        metadata: dict
    ) -> str:
        """Store a single content chunk with its embedding."""
        point_id = str(uuid.uuid4())

        point = PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "content": content,
                "module": metadata.get("module", ""),
                "chapter": metadata.get("chapter", ""),
                "section": metadata.get("section", ""),
                "heading_path": metadata.get("heading_path", ""),
                "source_url": metadata.get("source_url", ""),
                "created_at": metadata.get("created_at", ""),
            }
        )

        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

        return point_id

    async def store_embeddings(
        self,
        contents: List[str],
        embeddings: List[List[float]],
        metadata_list: List[dict]
    ) -> List[str]:
        """Store multiple content chunks with their embeddings."""
        point_ids = []
        points = []

        for content, embedding, metadata in zip(contents, embeddings, metadata_list):
            point_id = str(uuid.uuid4())
            point_ids.append(point_id)

            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": content,
                    "module": metadata.get("module", ""),
                    "chapter": metadata.get("chapter", ""),
                    "section": metadata.get("section", ""),
                    "heading_path": metadata.get("heading_path", ""),
                    "source_url": metadata.get("source_url", ""),
                    "created_at": metadata.get("created_at", ""),
                }
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return point_ids

    async def search_similar(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        filters: Optional[dict] = None
    ) -> List[dict]:
        """Search for similar content based on embedding."""
        # Build filter conditions if provided
        search_filter = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                if isinstance(value, list):
                    # Handle array values with 'is Any' condition
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchAny(any=value)
                        )
                    )
                else:
                    # Handle single values
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

            if filter_conditions:
                search_filter = models.Filter(
                    must=filter_conditions
                )

        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=top_k,
            with_payload=True,
            score_threshold=settings.similarity_threshold
        )

        return [
            {
                "id": result.id,
                "content": result.payload["content"],
                "module": result.payload["module"],
                "chapter": result.payload["chapter"],
                "section": result.payload["section"],
                "heading_path": result.payload["heading_path"],
                "source_url": result.payload["source_url"],
                "score": result.score
            }
            for result in results
        ]

    async def delete_by_source_url(self, source_url: str):
        """Delete all points with a specific source URL."""
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_url",
                            match=models.MatchValue(value=source_url)
                        )
                    ]
                )
            )
        )

    async def get_all_source_urls(self) -> List[str]:
        """Get all unique source URLs in the collection."""
        results = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="source_url",
                        match=models.MatchValue(value="")
                    )
                ]
            ),
            limit=10000,  # Adjust as needed
            with_payload=True
        )

        source_urls = set()
        for record, _ in results:
            if "source_url" in record.payload:
                source_urls.add(record.payload["source_url"])

        # Continue scrolling if there are more results
        while results[1] is not None:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_url",
                            match=models.MatchValue(value="")
                        )
                    ]
                ),
                offset=results[1],
                limit=10000,
                with_payload=True
            )
            for record, _ in results:
                if "source_url" in record.payload:
                    source_urls.add(record.payload["source_url"])

        return list(source_urls)

    async def health_check(self) -> bool:
        """Check if Qdrant is accessible."""
        try:
            self.client.get_collections()
            return True
        except Exception:
            return False


# Global instance
qdrant_service = QdrantService()