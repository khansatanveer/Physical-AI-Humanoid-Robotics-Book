from typing import List, Optional, Dict, Any
import json
import re
from enum import Enum

from clients.cohere_client import cohere_service
from models.book_content import SourceReference


class GenerationMode(Enum):
    """Enumeration for different generation modes."""
    GROUNDED = "grounded"
    SELECTION_ONLY = "selection_only"
    FREE_FORM = "free_form"


class GenerationUtils:
    """Utility class for text generation operations."""

    @staticmethod
    async def generate_answer_from_context(
        query: str,
        context: str,
        mode: GenerationMode = GenerationMode.GROUNDED,
        max_tokens: Optional[int] = 500,
        temperature: Optional[float] = 0.3
    ) -> str:
        """
        Generate an answer based on the provided context and query.

        Args:
            query: The user's question
            context: The context to base the answer on
            mode: The generation mode (affects prompt construction)
            max_tokens: Maximum tokens for the response
            temperature: Creativity level (0-1)

        Returns:
            Generated answer as a string
        """
        if mode == GenerationMode.SELECTION_ONLY:
            # For selection-only mode, explicitly instruct to only use provided context
            prompt = f"""
            You are a helpful assistant that answers questions based only on the provided text.
            Do not use any external knowledge or make assumptions beyond what is explicitly stated in the provided text.

            Provided text: {context}

            Question: {query}

            Answer: Please provide a direct answer based only on the information in the provided text.
            If the answer is not contained in the provided text, respond with: "The answer is not contained in the selected content."
            """
        elif mode == GenerationMode.GROUNDED:
            # For grounded mode, instruct to use only provided context
            prompt = f"""
            You are a helpful assistant that answers questions based only on the provided context.
            Do not use any external knowledge or make assumptions beyond what is explicitly stated in the provided context.

            Context: {context}

            Question: {query}

            Answer: Please provide a concise, accurate answer based only on the information in the provided context.
            If the answer cannot be found in the provided context, respond with: "The requested information is not present in the provided content."
            """
        else:  # FREE_FORM
            prompt = f"""
            Context: {context}

            Question: {query}

            Answer: Please provide a helpful answer based on the context provided.
            """

        response = await cohere_service.generate_response(
            prompt=prompt,
            max_tokens=max_tokens,
            temperature=temperature
        )

        return response

    @staticmethod
    async def generate_answer_with_sources(
        query: str,
        context_chunks: List[Dict[str, Any]],
        mode: GenerationMode = GenerationMode.GROUNDED,
        max_tokens: Optional[int] = 500,
        temperature: Optional[float] = 0.3
    ) -> tuple[str, List[SourceReference]]:
        """
        Generate an answer with source attribution.

        Args:
            query: The user's question
            context_chunks: List of context chunks with metadata
            mode: The generation mode
            max_tokens: Maximum tokens for the response
            temperature: Creativity level (0-1)

        Returns:
            Tuple of (answer, list of source references)
        """
        # Build context from chunks
        context_parts = []
        sources = []

        for chunk in context_chunks:
            content = chunk.get("content", "")
            context_parts.append(content)

            # Create source reference
            source = SourceReference(
                module=chunk.get("module", ""),
                chapter=chunk.get("chapter", ""),
                section=chunk.get("section", ""),
                heading_path=chunk.get("heading_path", ""),
                source_url=chunk.get("source_url", ""),
                text_preview=content[:200] + "..." if len(content) > 200 else content
            )
            sources.append(source)

        full_context = "\n\n".join(context_parts)

        answer = await GenerationUtils.generate_answer_from_context(
            query=query,
            context=full_context,
            mode=mode,
            max_tokens=max_tokens,
            temperature=temperature
        )

        return answer, sources

    @staticmethod
    async def validate_answer_grounding(
        answer: str,
        source_context: str,
        query: str
    ) -> tuple[bool, str]:
        """
        Validate that the answer is properly grounded in the source context.

        Args:
            answer: The generated answer
            source_context: The context used to generate the answer
            query: The original query

        Returns:
            Tuple of (is_valid, validation_message)
        """
        # Check if answer contains the specific refusal message
        if "not contained in the selected content" in answer.lower() or \
           "not present in the provided content" in answer.lower():
            return True, "Answer properly indicates content not found"

        # Check for basic grounding indicators
        answer_lower = answer.lower()
        context_lower = source_context.lower()

        # Look for common indicators of grounding
        has_grounding_indicators = any([
            phrase in answer_lower
            for phrase in [
                "according to", "as mentioned", "the text states",
                "based on", "the document says", "the content shows"
            ]
        ])

        # Check if answer contains information that appears in the context
        # This is a simple heuristic - in a production system, we'd want more sophisticated validation
        answer_sentences = re.split(r'[.!?]+', answer)
        context_sentences = re.split(r'[.!?]+', context_lower)

        # Look for sentence overlap as a basic grounding check
        sentence_overlap = 0
        for ans_sent in answer_sentences:
            ans_sent = ans_sent.strip().lower()
            if len(ans_sent) > 10:  # Only check sentences with meaningful content
                for ctx_sent in context_sentences:
                    if ans_sent in ctx_sent:
                        sentence_overlap += 1
                        break

        if sentence_overlap > 0 or has_grounding_indicators:
            return True, "Answer appears to be grounded in provided context"
        else:
            return False, "Answer may contain information not present in the provided context"

    @staticmethod
    async def format_sources_for_response(
        sources: List[SourceReference]
    ) -> str:
        """
        Format source references for inclusion in the response.

        Args:
            sources: List of source references

        Returns:
            Formatted string of sources
        """
        if not sources:
            return ""

        formatted_sources = ["\nSources referenced:"]
        for i, source in enumerate(sources, 1):
            formatted_sources.append(
                f"{i}. {source.module} > {source.chapter} > {source.section}"
            )
            if source.source_url:
                formatted_sources.append(f"   URL: {source.source_url}")

        return "\n".join(formatted_sources)

    @staticmethod
    async def generate_system_prompt(
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
        base_instructions = [
            "You are a helpful assistant for a technical book.",
            "Always provide accurate, concise answers.",
            "Structure responses clearly and logically."
        ]

        if mode == GenerationMode.SELECTION_ONLY:
            base_instructions.extend([
                "Answer only based on the provided selected text.",
                "Do not use any external knowledge.",
                "If the answer is not in the selected text, say: 'The answer is not contained in the selected content.'"
            ])
        elif mode == GenerationMode.GROUNDED:
            base_instructions.extend([
                "Answer only based on the provided context.",
                "Do not use any external knowledge.",
                "If the information is not in the provided context, say: 'The requested information is not present in the provided content.'"
            ])

        if custom_instructions:
            base_instructions.append(custom_instructions)

        return "\n".join(base_instructions)

    @staticmethod
    async def extract_entities_from_text(text: str) -> List[str]:
        """
        Extract key entities from text using pattern matching.
        In a production system, this would use more sophisticated NLP techniques.

        Args:
            text: Text to extract entities from

        Returns:
            List of extracted entities
        """
        import re

        # Simple patterns for common entities
        patterns = [
            r'\b[A-Z][a-z]+\b',  # Proper nouns
            r'\b\d+\.\d+\b',     # Version numbers
            r'\b[A-Z]{2,}\b',    # Acronyms
            r'\b[A-Za-z]+-[A-Za-z]+\b',  # Compound terms
        ]

        entities = set()
        for pattern in patterns:
            matches = re.findall(pattern, text)
            entities.update(matches)

        # Filter out common words that aren't likely entities
        common_words = {
            'The', 'And', 'For', 'Are', 'But', 'Not', 'You', 'All', 'Can', 'Had',
            'The', 'Was', 'To', 'Of', 'And', 'A', 'In', 'Is', 'It', 'On', 'That',
            'By', 'This', 'With', 'I', 'You', 'It', 'Or', 'Be', 'Have', 'From',
            'They', 'We', 'As', 'At', 'Do', 'At', 'This', 'But', 'His', 'By',
            'Would', 'Up', 'Out', 'If', 'About', 'Who', 'Get', 'Which', 'Go',
            'Me', 'When', 'Make', 'Can', 'Like', 'Time', 'No', 'Just', 'Him',
            'Know', 'Take', 'People', 'Into', 'Year', 'Your', 'Good', 'Some',
            'Could', 'Them', 'See', 'Other', 'Than', 'Then', 'Now', 'Look',
            'Only', 'Come', 'Its', 'Over', 'Think', 'Also', 'Back', 'After',
            'Use', 'Two', 'How', 'Our', 'Work', 'First', 'Well', 'Way', 'Even',
            'New', 'Want', 'Because', 'Any', 'These', 'Give', 'Day', 'Most', 'Us'
        }

        filtered_entities = [entity for entity in entities if entity not in common_words and len(entity) > 2]
        return sorted(filtered_entities, key=len, reverse=True)


# Convenience instance
generation_utils = GenerationUtils()