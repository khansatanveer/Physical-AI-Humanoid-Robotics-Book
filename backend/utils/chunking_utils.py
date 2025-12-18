import re
from typing import List, Tuple, Optional
from dataclasses import dataclass
from config import settings


@dataclass
class ContentChunk:
    """Represents a chunk of content with its metadata."""
    text: str
    start_pos: int
    end_pos: int
    chunk_index: int
    heading_context: str = ""


class ChunkingUtils:
    """Utility class for deterministic content chunking."""

    @staticmethod
    def chunk_by_tokens(
        text: str,
        max_tokens: int = 512,
        overlap_tokens: int = 128,
        preserve_paragraphs: bool = True
    ) -> List[ContentChunk]:
        """
        Deterministically chunk text based on token count with overlap.

        Args:
            text: The text to chunk
            max_tokens: Maximum tokens per chunk (approximately characters for simplicity)
            overlap_tokens: Number of tokens to overlap between chunks
            preserve_paragraphs: Whether to try to preserve paragraph boundaries

        Returns:
            List of ContentChunk objects
        """
        if not text.strip():
            return []

        # For this implementation, we'll use a character-based approach
        # In a production system, we'd use proper tokenization (e.g., with tiktoken)
        max_chars = max_tokens  # Approximate token to character ratio
        overlap_chars = overlap_tokens  # Approximate token to character ratio

        chunks = []
        start_pos = 0
        chunk_index = 0

        while start_pos < len(text):
            # Determine the end position
            end_pos = start_pos + max_chars

            # If we're at the end of the text, adjust the end position
            if end_pos >= len(text):
                end_pos = len(text)
            else:
                # Try to break at a sentence or paragraph boundary if preserving paragraphs
                if preserve_paragraphs:
                    # Look for paragraph breaks first
                    paragraph_pos = text.rfind('\n\n', start_pos + max_chars // 2, start_pos + max_chars + overlap_chars)
                    sentence_pos = text.rfind('. ', start_pos + max_chars // 2, start_pos + max_chars + overlap_chars)
                    space_pos = text.rfind(' ', start_pos + max_chars // 2, start_pos + max_chars + overlap_chars)

                    # Prioritize paragraph breaks, then sentences, then spaces
                    best_pos = max(
                        pos for pos in [paragraph_pos, sentence_pos, space_pos]
                        if pos != -1
                    )

                    if best_pos > start_pos + max_chars // 2:
                        end_pos = best_pos + 1  # Include the delimiter

            # Extract the chunk text
            chunk_text = text[start_pos:end_pos].strip()

            if chunk_text:  # Only add non-empty chunks
                chunk = ContentChunk(
                    text=chunk_text,
                    start_pos=start_pos,
                    end_pos=end_pos,
                    chunk_index=chunk_index
                )
                chunks.append(chunk)

            # Move to the next position, accounting for overlap
            if end_pos >= len(text):
                # At the end, no more chunks needed
                break

            # Calculate next start position with overlap
            next_start = end_pos - overlap_chars if overlap_chars < end_pos else start_pos + 1

            # Ensure we make progress to avoid infinite loops
            if next_start <= start_pos:
                next_start = start_pos + 1

            start_pos = next_start
            chunk_index += 1

        return chunks

    @staticmethod
    def chunk_by_headings(
        text: str,
        max_tokens: int = 512,
        overlap_tokens: int = 128
    ) -> List[ContentChunk]:
        """
        Chunk text by headings, respecting max token limits.

        Args:
            text: The text to chunk
            max_tokens: Maximum tokens per chunk
            overlap_tokens: Number of tokens to overlap between chunks

        Returns:
            List of ContentChunk objects
        """
        # Find all headings (h1, h2, h3, etc. patterns)
        heading_pattern = r'^(#{1,6})\s+(.+)$|^(.+)\n={3,}$|^(.+)\n-{3,}$'
        lines = text.split('\n')

        chunks = []
        current_chunk = []
        current_heading = ""
        chunk_char_count = 0
        start_pos = 0
        chunk_index = 0

        i = 0
        while i < len(lines):
            line = lines[i]
            line_char_count = len(line)

            # Check if this line is a heading
            heading_match = re.match(heading_pattern, line, re.MULTILINE)
            if heading_match:
                # If we have accumulated content and it's substantial, save the chunk
                if current_chunk and chunk_char_count > 0:
                    chunk_text = '\n'.join(current_chunk).strip()
                    if len(chunk_text) > 0:
                        # If the chunk is too large, split it further
                        if len(chunk_text) > max_tokens:
                            sub_chunks = ChunkingUtils._split_large_chunk(
                                chunk_text, max_tokens, overlap_tokens
                            )
                            for sub_chunk in sub_chunks:
                                sub_chunk.heading_context = current_heading
                                chunks.append(sub_chunk)
                        else:
                            chunk = ContentChunk(
                                text=chunk_text,
                                start_pos=start_pos,
                                end_pos=start_pos + len(chunk_text),
                                chunk_index=chunk_index,
                                heading_context=current_heading
                            )
                            chunks.append(chunk)
                            chunk_index += 1

                # Update the current heading
                if heading_match.group(2):  # Markdown heading like # Heading
                    current_heading = heading_match.group(2)
                elif heading_match.group(3):  # Setext heading with ===
                    current_heading = heading_match.group(3)
                elif heading_match.group(4):  # Setext heading with ---
                    current_heading = heading_match.group(4)

                current_chunk = [line]
                chunk_char_count = line_char_count
                start_pos = sum(len(l) + 1 for l in lines[:i])  # Account for newlines
            else:
                # Add line to current chunk if it doesn't exceed limits
                if chunk_char_count + line_char_count + 1 <= max_tokens or not current_chunk:
                    current_chunk.append(line)
                    chunk_char_count += line_char_count + 1  # +1 for newline
                else:
                    # Save the current chunk
                    chunk_text = '\n'.join(current_chunk).strip()
                    if chunk_text:
                        chunk = ContentChunk(
                            text=chunk_text,
                            start_pos=start_pos,
                            end_pos=start_pos + len(chunk_text),
                            chunk_index=chunk_index,
                            heading_context=current_heading
                        )
                        chunks.append(chunk)
                        chunk_index += 1

                    # Start a new chunk with the current line
                    current_chunk = [line]
                    chunk_char_count = line_char_count
                    start_pos = sum(len(l) + 1 for l in lines[:i])

            i += 1

        # Add the final chunk if there's content left
        if current_chunk:
            chunk_text = '\n'.join(current_chunk).strip()
            if chunk_text:
                if len(chunk_text) > max_tokens:
                    sub_chunks = ChunkingUtils._split_large_chunk(
                        chunk_text, max_tokens, overlap_tokens
                    )
                    for sub_chunk in sub_chunks:
                        sub_chunk.heading_context = current_heading
                        chunks.append(sub_chunk)
                else:
                    chunk = ContentChunk(
                        text=chunk_text,
                        start_pos=start_pos,
                        end_pos=start_pos + len(chunk_text),
                        chunk_index=chunk_index,
                        heading_context=current_heading
                    )
                    chunks.append(chunk)

        return chunks

    @staticmethod
    def _split_large_chunk(
        chunk_text: str,
        max_tokens: int,
        overlap_tokens: int
    ) -> List[ContentChunk]:
        """
        Split a large chunk into smaller ones.

        Args:
            chunk_text: The large chunk to split
            max_tokens: Maximum tokens per sub-chunk
            overlap_tokens: Number of tokens to overlap between sub-chunks

        Returns:
            List of ContentChunk objects
        """
        # Use the basic token-based chunking for splitting large chunks
        temp_chunks = ChunkingUtils.chunk_by_tokens(
            chunk_text,
            max_tokens=max_tokens,
            overlap_tokens=overlap_tokens,
            preserve_paragraphs=True
        )

        # Adjust positions to be relative to the original text
        result = []
        for i, chunk in enumerate(temp_chunks):
            result.append(ContentChunk(
                text=chunk.text,
                start_pos=chunk.start_pos,
                end_pos=chunk.end_pos,
                chunk_index=i
            ))

        return result

    @staticmethod
    def validate_chunking(
        original_text: str,
        chunks: List[ContentChunk]
    ) -> Tuple[bool, str]:
        """
        Validate that the chunking preserves the original text.

        Args:
            original_text: The original text that was chunked
            chunks: The resulting chunks

        Returns:
            Tuple of (is_valid, validation_message)
        """
        if not chunks:
            return True, "No chunks to validate"

        # Reconstruct the text from chunks
        reconstructed = ""
        expected_pos = 0

        for chunk in chunks:
            if chunk.start_pos != expected_pos:
                return False, f"Chunk at index {chunk.chunk_index} has incorrect start position"

            reconstructed += chunk.text

            # Check for overlap
            if chunk.chunk_index < len(chunks) - 1:
                # For overlapping chunks, the next chunk should start within the current chunk
                next_chunk = chunks[chunk.chunk_index + 1]
                if next_chunk.start_pos > chunk.end_pos:
                    # There's a gap between chunks
                    gap_text = original_text[chunk.end_pos:next_chunk.start_pos]
                    reconstructed += gap_text

            expected_pos = chunk.end_pos

        # For overlapping chunks, we can't do a perfect reconstruction
        # Instead, check that all chunks are substrings of the original
        for chunk in chunks:
            if chunk.text not in original_text:
                return False, f"Chunk at index {chunk.chunk_index} is not in original text"

        return True, f"Valid chunking with {len(chunks)} chunks created"

    @staticmethod
    def get_optimal_chunking_strategy(
        content_type: str,
        content: str
    ) -> str:
        """
        Determine the optimal chunking strategy based on content type.

        Args:
            content_type: Type of content ("markdown", "text", "code", etc.)
            content: The content to be chunked

        Returns:
            Recommended chunking strategy ("by_tokens", "by_headings", etc.)
        """
        if content_type.lower() in ["markdown", "md", "mdx"]:
            # For markdown content, try heading-based chunking first
            heading_pattern = r'^(#{1,6})\s+(.+)$|^(.+)\n={3,}$|^(.+)\n-{3,}$'
            lines = content.split('\n')
            heading_count = sum(1 for line in lines if re.match(heading_pattern, line, re.MULTILINE))

            if heading_count > 0:
                return "by_headings"

        return "by_tokens"


# Convenience instance
chunking_utils = ChunkingUtils()