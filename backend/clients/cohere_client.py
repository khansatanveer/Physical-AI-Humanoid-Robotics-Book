import cohere
from typing import List, Optional
import asyncio

from config import settings


class CohereService:
    """Service class to handle Cohere API operations."""

    def __init__(self):
        """Initialize Cohere client with API key."""
        self.client = cohere.Client(settings.cohere_api_key)
        self.embed_model = "embed-multilingual-v3.0"  # Recommended for mixed languages
        self.generate_model = "command-r-plus"  # Recommended for RAG applications

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts using Cohere."""
        loop = asyncio.get_event_loop()

        def sync_embeddings():
            response = self.client.embed(
                texts=texts,
                model=self.embed_model,
                input_type="search_document"  # Optimal for document search
            )
            return response.embeddings

        embeddings = await loop.run_in_executor(None, sync_embeddings)
        return embeddings

    async def generate_embeddings_single(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        embeddings = await self.generate_embeddings([text])
        return embeddings[0] if embeddings else []

    async def generate_response(
        self,
        prompt: str,
        max_tokens: Optional[int] = 500,
        temperature: Optional[float] = 0.3,
        preamble: Optional[str] = None
    ) -> str:
        """Generate a response using Cohere's language model."""
        loop = asyncio.get_event_loop()

        def sync_generate():
            response = self.client.generate(
                model=self.generate_model,
                prompt=prompt,
                max_tokens=max_tokens,
                temperature=temperature,
                stop_sequences=["\n\n", "User:", "Assistant:"],
                return_likelihoods="NONE"
            )
            return response.generations[0].text if response.generations else ""

        result = await loop.run_in_executor(None, sync_generate)
        return result.strip()

    async def chat_response(
        self,
        message: str,
        chat_history: Optional[List[dict]] = None,
        max_tokens: Optional[int] = 500,
        temperature: Optional[float] = 0.3
    ) -> str:
        """Generate a chat response using Cohere's chat model."""
        loop = asyncio.get_event_loop()

        def sync_chat():
            response = self.client.chat(
                model=self.generate_model,
                message=message,
                chat_history=chat_history or [],
                max_tokens=max_tokens,
                temperature=temperature
            )
            return response.text

        result = await loop.run_in_executor(None, sync_chat)
        return result.strip()

    async def classify_text(
        self,
        inputs: List[str],
        examples: List[dict]
    ) -> List[str]:
        """Classify text using Cohere's classification model."""
        loop = asyncio.get_event_loop()

        def sync_classify():
            response = self.client.classify(
                inputs=inputs,
                examples=examples
            )
            return [classification.prediction for classification in response.classifications]

        result = await loop.run_in_executor(None, sync_classify)
        return result

    async def detect_language(self, texts: List[str]) -> List[str]:
        """Detect language of texts using Cohere."""
        loop = asyncio.get_event_loop()

        def sync_detect():
            response = self.client.detect_language(texts=texts)
            return [lang.code for lang in response.results]

        result = await loop.run_in_executor(None, sync_detect)
        return result

    async def health_check(self) -> bool:
        """Check if Cohere API is accessible by making a simple request."""
        try:
            # Test with a simple embedding request
            test_embedding = await self.generate_embeddings_single("health check")
            return len(test_embedding) > 0
        except Exception:
            return False


# Global instance
cohere_service = CohereService()