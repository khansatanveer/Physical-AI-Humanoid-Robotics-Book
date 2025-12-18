from typing import List, Optional
import asyncio
from functools import lru_cache

from clients.cohere_client import cohere_service


class EmbeddingUtils:
    """Utility class for embedding operations."""

    @staticmethod
    async def generate_embeddings(texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        if not texts:
            return []

        # Limit batch size to avoid API limits
        batch_size = 96  # Cohere's recommended batch size
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_embeddings = await cohere_service.generate_embeddings(batch)
            all_embeddings.extend(batch_embeddings)

        return all_embeddings

    @staticmethod
    async def generate_single_embedding(text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            Embedding as a list of floats
        """
        if not text.strip():
            return []

        return await cohere_service.generate_embeddings_single(text)

    @staticmethod
    async def calculate_similarity(
        embedding1: List[float],
        embedding2: List[float]
    ) -> float:
        """
        Calculate cosine similarity between two embeddings.

        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector

        Returns:
            Cosine similarity score between 0 and 1
        """
        if not embedding1 or not embedding2 or len(embedding1) != len(embedding2):
            return 0.0

        # Calculate dot product
        dot_product = sum(a * b for a, b in zip(embedding1, embedding2))

        # Calculate magnitudes
        magnitude1 = sum(a * a for a in embedding1) ** 0.5
        magnitude2 = sum(b * b for b in embedding2) ** 0.5

        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        # Calculate cosine similarity
        similarity = dot_product / (magnitude1 * magnitude2)

        # Ensure the result is between 0 and 1 (cosine similarity can be negative)
        return max(0.0, min(1.0, (similarity + 1) / 2))

    @staticmethod
    async def find_most_similar(
        query_embedding: List[float],
        candidate_embeddings: List[List[float]],
        top_k: int = 5
    ) -> List[tuple]:
        """
        Find the most similar embeddings to the query embedding.

        Args:
            query_embedding: Embedding to compare against
            candidate_embeddings: List of candidate embeddings
            top_k: Number of top results to return

        Returns:
            List of tuples (index, similarity_score) sorted by similarity
        """
        similarities = []

        for i, candidate_embedding in enumerate(candidate_embeddings):
            similarity = await EmbeddingUtils.calculate_similarity(
                query_embedding,
                candidate_embedding
            )
            similarities.append((i, similarity))

        # Sort by similarity score in descending order
        similarities.sort(key=lambda x: x[1], reverse=True)

        return similarities[:top_k]

    @staticmethod
    @lru_cache(maxsize=1000)
    def _cached_normalize_text(text: str) -> str:
        """
        Internal method to normalize text for consistent embedding generation.
        Uses LRU cache for performance.
        """
        import re

        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)
        # Strip leading/trailing whitespace
        text = text.strip()
        return text

    @staticmethod
    async def normalize_and_embed(text: str) -> List[float]:
        """
        Normalize text and generate embedding.

        Args:
            text: Text to normalize and embed

        Returns:
            Normalized embedding as a list of floats
        """
        normalized_text = EmbeddingUtils._cached_normalize_text(text)
        return await EmbeddingUtils.generate_single_embedding(normalized_text)

    @staticmethod
    async def batch_normalize_and_embed(texts: List[str]) -> List[List[float]]:
        """
        Normalize texts and generate embeddings in batch.

        Args:
            texts: List of texts to normalize and embed

        Returns:
            List of embeddings as lists of floats
        """
        normalized_texts = [EmbeddingUtils._cached_normalize_text(text) for text in texts]
        return await EmbeddingUtils.generate_embeddings(normalized_texts)


# Convenience instance
embedding_utils = EmbeddingUtils()