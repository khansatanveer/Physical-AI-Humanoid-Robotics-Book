import qdrant_client
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import logging
from config import Config
import hashlib

class StorageManager:
    """Utility for storing embeddings in Qdrant vector database."""

    def __init__(self):
        if not Config.QDRANT_URL:
            raise ValueError("QDRANT_URL is required for storage")

        logging.info(f"Initializing StorageManager with Qdrant at {Config.QDRANT_URL}")

        # Initialize Qdrant client
        self.client = qdrant_client.QdrantClient(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            timeout=30
        )

        self.collection_name = Config.QDRANT_COLLECTION_NAME
        logging.info(f"StorageManager initialized for collection: {self.collection_name}")

    def setup_collection(self):
        """Set up the Qdrant collection for storing embeddings."""
        logging.info(f"Setting up Qdrant collection: {self.collection_name}")
        try:
            # Check if collection already exists
            logging.info("Checking if Qdrant collection exists...")
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                logging.info(f"Creating new Qdrant collection: {self.collection_name}")
                # Create collection with appropriate vector size (Cohere embeddings are 1024-dimensional)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Cohere multilingual-v3.0 embeddings are 1024-dimensional
                        distance=models.Distance.COSINE
                    )
                )
                logging.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logging.info(f"Qdrant collection {self.collection_name} already exists")

            # Create payload index for efficient filtering
            logging.info("Creating payload index for 'url' field...")
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="url",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            logging.info("Creating payload index for 'content_hash' field...")
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="content_hash",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            logging.info(f"Successfully set up Qdrant collection: {self.collection_name}")
        except qdrant_client.http.exceptions.UnexpectedResponse as e:
            logging.error(f"Qdrant service error during setup: {str(e)}")
            raise
        except requests.exceptions.ConnectionError:
            logging.error(f"Connection error connecting to Qdrant at {Config.QDRANT_URL}")
            raise
        except requests.exceptions.Timeout:
            logging.error(f"Timeout connecting to Qdrant at {Config.QDRANT_URL}")
            raise
        except Exception as e:
            logging.error(f"Error setting up Qdrant collection: {str(e)}")
            raise

    def check_duplicate(self, content_hash: str) -> bool:
        """Check if content with given hash already exists in Qdrant."""
        logging.debug(f"Checking for duplicate content with hash: {content_hash[:16]}...")
        try:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="metadata.content_hash",
                            match=models.MatchValue(value=content_hash)
                        )
                    ]
                ),
                limit=1
            )
            is_duplicate = len(results[0]) > 0 if results[0] else False
            logging.debug(f"Duplicate check result for hash {content_hash[:16]}...: {is_duplicate}")
            return is_duplicate
        except Exception as e:
            logging.error(f"Error checking for duplicate with hash {content_hash}: {str(e)}")
            return False  # If we can't check, assume it's not duplicate to be safe

    def check_content_change(self, url: str, new_content_hash: str) -> tuple[bool, Optional[str]]:
        """Check if content for a URL has changed by comparing hashes.

        Returns:
            tuple: (has_changed, existing_content_hash) where has_changed is True if content has changed
        """
        logging.info(f"Checking content change for URL: {url}")
        try:
            # Find all records for this URL
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="url",
                            match=models.MatchValue(value=url)
                        )
                    ]
                ),
                limit=100  # Assuming a URL won't have more than 100 chunks
            )

            if not results[0]:  # No existing content for this URL
                logging.info(f"No existing content found for URL {url}, content is new")
                return True, None  # Content is new, so it has "changed" (first time)

            # Check if any of the existing chunks have different content hashes
            existing_hashes = set()
            for point in results[0]:
                content_hash = point.payload.get('metadata', {}).get('content_hash')
                if content_hash:
                    existing_hashes.add(content_hash)

            # If the new content hash is not in the existing hashes, content has changed
            has_changed = new_content_hash not in existing_hashes
            logging.info(f"Content change check for {url}: {'changed' if has_changed else 'unchanged'}")

            # Return the first existing hash as reference (or None if no existing hashes)
            existing_hash = next(iter(existing_hashes)) if existing_hashes else None

            return has_changed, existing_hash

        except Exception as e:
            logging.error(f"Error checking content change for URL {url}: {str(e)}")
            # In case of error, assume content has changed to be safe
            return True, None

    def store_embeddings(self, chunk_embeddings: List[Dict]):
        """Store embeddings in Qdrant with metadata."""
        logging.info(f"Starting storage of {len(chunk_embeddings)} embeddings in Qdrant")
        if not chunk_embeddings:
            logging.warning("No embeddings to store")
            return

        points = []
        for i, chunk_data in enumerate(chunk_embeddings):
            # Create a unique ID based on content hash and chunk ID
            content_hash = chunk_data.get('metadata', {}).get('content_hash', '')
            chunk_id = chunk_data.get('id', '')
            point_id = hashlib.sha256(f"{content_hash}_{chunk_id}".encode()).hexdigest()[:16]

            # Prepare the payload with all metadata
            payload = {
                'url': chunk_data.get('url', ''),
                'title': chunk_data.get('title', ''),
                'headings': chunk_data.get('headings', []),
                'content': chunk_data.get('content', ''),
                'metadata': chunk_data.get('metadata', {})
            }

            # Get the embedding vector
            vector = chunk_data.get('embedding', [])

            # Create a Qdrant point
            point = models.PointStruct(
                id=point_id,
                vector=vector,
                payload=payload
            )

            points.append(point)

            if i % 10 == 0:  # Log progress every 10 points
                logging.debug(f"Processed {i+1}/{len(chunk_embeddings)} embedding points for storage")

        logging.info(f"Created {len(points)} Qdrant points, preparing to upsert to collection {self.collection_name}")

        # Upsert points to Qdrant
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logging.info(f"Successfully stored {len(points)} embeddings in Qdrant collection {self.collection_name}")
        except Exception as e:
            logging.error(f"Error storing embeddings in Qdrant: {str(e)}")
            raise

    def idempotent_store_embeddings(self, chunk_embeddings: List[Dict]):
        """Store embeddings with idempotency - skip if already exists."""
        logging.info(f"Starting idempotent storage of {len(chunk_embeddings)} embeddings")
        if not chunk_embeddings:
            logging.warning("No embeddings to store idempotently")
            return

        # Filter out embeddings that already exist
        filtered_chunks = []
        skipped_count = 0

        for i, chunk_data in enumerate(chunk_embeddings):
            content_hash = chunk_data.get('metadata', {}).get('content_hash', '')
            if not self.check_duplicate(content_hash):
                filtered_chunks.append(chunk_data)
            else:
                logging.info(f"Skipping duplicate content with hash: {content_hash[:16]}...")
                skipped_count += 1

            if i % 10 == 0:  # Log progress
                logging.debug(f"Checked {i+1}/{len(chunk_embeddings)} embeddings for duplicates")

        logging.info(f"Filtered out {skipped_count} duplicate embeddings, storing {len(filtered_chunks)} new embeddings")

        if filtered_chunks:
            self.store_embeddings(filtered_chunks)
        else:
            logging.info("No new content to store - all content already exists in Qdrant")

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict]:
        """Search for similar content in Qdrant."""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            return [
                {
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score
                }
                for result in results
            ]
        except qdrant_client.http.exceptions.UnexpectedResponse as e:
            logging.error(f"Qdrant service error during search: {str(e)}")
            return []
        except Exception as e:
            logging.error(f"Error searching in Qdrant: {str(e)}")
            return []