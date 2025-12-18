from typing import List, Optional, Dict, Any
import asyncio
from datetime import datetime
import re

from clients.cohere_client import cohere_service
from utils.generation_utils import generation_utils, GenerationMode
from models.book_content import SourceReference
from config import settings


class GenerationService:
    """Service class for handling text generation with grounding enforcement."""

    def __init__(self):
        """Initialize the generation service with required clients."""
        self.cohere_service = cohere_service
        self.generation_utils = generation_utils

    async def generate_ground_response(
        self,
        query: str,
        context: str,
        mode: GenerationMode = GenerationMode.GROUNDED,
        max_tokens: Optional[int] = 500,
        temperature: Optional[float] = 0.3,
        validate_grounding: bool = True
    ) -> str:
        """
        Generate a response that is strictly grounded in the provided context.

        Args:
            query: The user's question
            context: The context to base the answer on
            mode: The generation mode (affects prompt construction)
            max_tokens: Maximum tokens for the response
            temperature: Creativity level (0-1)
            validate_grounding: Whether to validate the response is grounded

        Returns:
            Generated response string
        """
        response = await self.generation_utils.generate_answer_from_context(
            query=query,
            context=context,
            mode=mode,
            max_tokens=max_tokens,
            temperature=temperature
        )

        # Optionally validate that the response is properly grounded
        if validate_grounding and mode in [GenerationMode.GROUNDED, GenerationMode.SELECTION_ONLY]:
            is_valid, validation_msg = await self.generation_utils.validate_answer_grounding(
                answer=response,
                source_context=context,
                query=query
            )

            if not is_valid and "not contained" not in response.lower() and "not present" not in response.lower():
                # If validation fails and the response doesn't indicate content is missing,
                # try again with stricter grounding
                response = await self._generate_with_strict_grounding(
                    query=query,
                    context=context,
                    mode=mode,
                    max_tokens=max_tokens,
                    temperature=temperature
                )

        return response

    async def _generate_with_strict_grounding(
        self,
        query: str,
        context: str,
        mode: GenerationMode,
        max_tokens: Optional[int],
        temperature: Optional[float]
    ) -> str:
        """
        Generate response with stricter grounding enforcement.

        Args:
            query: The user's question
            context: The context to base the answer on
            mode: The generation mode
            max_tokens: Maximum tokens for the response
            temperature: Creativity level (0-1)

        Returns:
            Generated response string with strict grounding
        """
        # Create a more explicit prompt for strict grounding
        if mode == GenerationMode.SELECTION_ONLY:
            prompt = f"""
            IMPORTANT: Answer only based on the provided text below. Do not use any external knowledge.
            If the answer is not in the provided text, respond with: "The answer is not contained in the selected content."

            PROVIDED TEXT:
            {context}

            QUESTION: {query}

            ANSWER: Provide a direct answer based ONLY on the information in the provided text.
            """
        else:  # GROUNDED mode
            prompt = f"""
            IMPORTANT: Answer only based on the context provided below. Do not use any external knowledge.
            If the requested information is not in the provided context, respond with: "The requested information is not present in the provided content."

            CONTEXT:
            {context}

            QUESTION: {query}

            ANSWER: Provide a concise, accurate answer based ONLY on the information in the provided context.
            """

        return await self.cohere_service.generate_response(
            prompt=prompt,
            max_tokens=max_tokens,
            temperature=temperature
        )

    async def generate_response_with_sources(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        mode: GenerationMode = GenerationMode.GROUNDED,
        max_tokens: Optional[int] = 500,
        temperature: Optional[float] = 0.3
    ) -> tuple[str, List[SourceReference]]:
        """
        Generate a response with source attribution.

        Args:
            query: The user's question
            context_chunks: List of context chunks with metadata
            mode: The generation mode
            max_tokens: Maximum tokens for the response
            temperature: Creativity level (0-1)

        Returns:
            Tuple of (response, list of source references)
        """
        return await self.generation_utils.generate_answer_with_sources(
            query=query,
            context_chunks=context_chunks,
            mode=mode,
            max_tokens=max_tokens,
            temperature=temperature
        )

    async def generate_summarization(
        self,
        content: str,
        max_tokens: Optional[int] = 200,
        temperature: Optional[float] = 0.2
    ) -> str:
        """
        Generate a summary of the provided content.

        Args:
            content: Content to summarize
            max_tokens: Maximum tokens for the summary
            temperature: Creativity level (0-1)

        Returns:
            Generated summary
        """
        prompt = f"""
        Please provide a concise summary of the following content:

        {content}

        SUMMARY:
        """

        summary = await self.cohere_service.generate_response(
            prompt=prompt,
            max_tokens=max_tokens,
            temperature=temperature
        )

        return summary

    async def generate_explanation(
        self,
        concept: str,
        context: str,
        max_tokens: Optional[int] = 300,
        temperature: Optional[float] = 0.3
    ) -> str:
        """
        Generate an explanation of a concept based on context.

        Args:
            concept: The concept to explain
            context: Context to base the explanation on
            max_tokens: Maximum tokens for the explanation
            temperature: Creativity level (0-1)

        Returns:
            Generated explanation
        """
        prompt = f"""
        Based on the following context, please explain the concept of "{concept}":

        CONTEXT:
        {context}

        EXPLANATION: Provide a clear explanation of {concept} based only on the provided context.
        """

        explanation = await self.cohere_service.generate_response(
            prompt=prompt,
            max_tokens=max_tokens,
            temperature=temperature
        )

        return explanation

    async def generate_quiz_questions(
        self,
        content: str,
        num_questions: int = 3,
        difficulty: str = "medium"
    ) -> str:
        """
        Generate quiz questions based on content.

        Args:
            content: Content to generate questions from
            num_questions: Number of questions to generate
            difficulty: Difficulty level ("easy", "medium", "hard")

        Returns:
            Generated quiz questions
        """
        difficulty_prompt = {
            "easy": "Focus on main concepts and definitions",
            "medium": "Include application of concepts",
            "hard": "Include analysis and synthesis of concepts"
        }

        prompt = f"""
        Based on the following content, generate {num_questions} quiz questions at {difficulty} difficulty.
        {difficulty_prompt.get(difficulty, "medium")}.
        Include answers after each question.

        CONTENT:
        {content}

        QUESTIONS AND ANSWERS:
        """

        questions = await self.cohere_service.generate_response(
            prompt=prompt,
            max_tokens=500,
            temperature=0.7
        )

        return questions

    async def validate_response_grounding(
        self,
        response: str,
        source_context: str,
        query: str
    ) -> tuple[bool, str]:
        """
        Validate that a response is properly grounded in the source context.

        Args:
            response: The generated response to validate
            source_context: The context used to generate the response
            query: The original query

        Returns:
            Tuple of (is_valid, validation_message)
        """
        return await self.generation_utils.validate_answer_grounding(
            answer=response,
            source_context=source_context,
            query=query
        )

    async def enforce_zero_hallucination(
        self,
        response: str,
        source_context: str
    ) -> str:
        """
        Post-process a response to minimize hallucinations.

        Args:
            response: The response to check for hallucinations
            source_context: The source context

        Returns:
            Response with hallucinations minimized
        """
        # Check if response contains the specific refusal messages
        if "not contained in the selected content" in response.lower() or \
           "not present in the provided content" in response.lower():
            return response  # Already properly indicates content not found

        # Look for phrases that indicate external knowledge
        external_indicators = [
            r'\bknown\s+to\s+be\b',
            r'\bgenerally\s+accepted\b',
            r'\bcommonly\s+known\b',
            r'\bmany\s+experts\s+agree\b',
            r'\bit\'?s\s+well\s+known\b'
        ]

        for indicator in external_indicators:
            if re.search(indicator, response, re.IGNORECASE):
                # If external knowledge indicators are found, return a grounding reminder
                return "The requested information is not present in the provided content."

        # For more sophisticated hallucination detection, we would need to
        # compare claims in the response with the source context
        # This is a simplified implementation

        return response

    async def format_response_with_citations(
        self,
        response: str,
        sources: List[SourceReference]
    ) -> str:
        """
        Format a response with proper citations to sources.

        Args:
            response: The response to format
            sources: List of sources used in the response

        Returns:
            Formatted response with citations
        """
        formatted_response = response

        if sources:
            formatted_response += await self.generation_utils.format_sources_for_response(sources)

        return formatted_response

    async def generate_system_prompt(
        self,
        mode: GenerationMode,
        custom_instructions: Optional[str] = None
    ) -> str:
        """
        Generate a system prompt based on the mode and custom instructions.

        Args:
            mode: The generation mode
            custom_instructions: Additional custom instructions

        Returns:
            System prompt string
        """
        return await self.generation_utils.generate_system_prompt(
            mode=mode,
            custom_instructions=custom_instructions
        )

    async def extract_key_entities(
        self,
        text: str
    ) -> List[str]:
        """
        Extract key entities from text.

        Args:
            text: Text to extract entities from

        Returns:
            List of extracted entities
        """
        return await self.generation_utils.extract_entities_from_text(text)

    async def health_check(self) -> bool:
        """
        Perform a health check of the generation service.

        Returns:
            True if all dependencies are healthy, False otherwise
        """
        return await self.cohere_service.health_check()


# Global instance
generation_service = GenerationService()