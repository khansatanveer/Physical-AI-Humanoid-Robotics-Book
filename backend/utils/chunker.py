import logging
from typing import List, Dict
import re

class Chunker:
    """Content chunking utility for splitting book content into manageable pieces."""

    def __init__(self, chunk_size: int = 1000, overlap_size: int = 100):
        self.chunk_size = chunk_size
        self.overlap_size = overlap_size

    def chunk_content(self, content_data: Dict) -> List[Dict]:
        """Split content into chunks while preserving metadata and structure."""
        content = content_data.get('content', '')
        if not content:
            logging.info("Content is empty, returning empty chunks list")
            return []

        logging.info(f"Starting chunking process for content with {len(content)} characters")

        # Validate content size to prevent exceeding embedding limits
        max_content_size = 30000  # Conservative limit for Cohere API
        if len(content) > max_content_size:
            logging.warning(f"Content exceeds maximum size of {max_content_size} characters. Truncating...")
            content = content[:max_content_size]
            logging.info(f"Content truncated to {len(content)} characters")

        # Split content into chunks using a sliding window approach
        chunks = []
        content_length = len(content)

        start_idx = 0
        while start_idx < content_length:
            end_idx = start_idx + self.chunk_size

            # If this is the last chunk and it's smaller than chunk_size, include it all
            if end_idx >= content_length:
                end_idx = content_length
            else:
                # Try to break at sentence boundary to avoid cutting sentences
                chunk = content[start_idx:end_idx]

                # Find the last sentence boundary within the chunk
                sentence_end = max(
                    chunk.rfind('. '),
                    chunk.rfind('! '),
                    chunk.rfind('? '),
                    chunk.rfind('\n'),
                    chunk.rfind('.\n'),
                    chunk.rfind('!'),
                    chunk.rfind('?')
                )

                if sentence_end > len(chunk) // 2:  # Only break if sentence end is in the second half of the chunk
                    end_idx = start_idx + sentence_end + 1
                else:
                    # If no good sentence boundary found, just use the full chunk size
                    end_idx = start_idx + self.chunk_size
                    if end_idx > content_length:
                        end_idx = content_length

            chunk_text = content[start_idx:end_idx].strip()
            if chunk_text:  # Only add non-empty chunks
                # Validate chunk size to prevent exceeding embedding limits
                max_chunk_size = 20000  # Conservative limit for Cohere API
                if len(chunk_text) > max_chunk_size:
                    logging.warning(f"Chunk exceeds maximum size of {max_chunk_size} characters. Splitting further...")
                    # Split the large chunk into smaller ones
                    sub_chunks = self._split_large_chunk(chunk_text, max_chunk_size)
                    for i, sub_chunk in enumerate(sub_chunks):
                        sub_chunk_id = f"{content_data.get('metadata', {}).get('content_hash', '')[:16]}_{start_idx}_{end_idx}_{i}"
                        chunk_metadata = content_data.get('metadata', {}).copy()
                        chunk_metadata.update({
                            'chunk_id': sub_chunk_id,
                            'start_idx': start_idx,
                            'end_idx': end_idx,
                            'chunk_size': len(sub_chunk),
                            'total_content_length': content_length,
                            'is_sub_chunk': True
                        })

                        chunk_entry = {
                            'id': sub_chunk_id,
                            'url': content_data.get('url', ''),
                            'title': content_data.get('title', ''),
                            'headings': content_data.get('headings', []),
                            'content': sub_chunk,
                            'metadata': chunk_metadata
                        }
                        chunks.append(chunk_entry)
                else:
                    chunk_id = f"{content_data.get('metadata', {}).get('content_hash', '')[:16]}_{start_idx}_{end_idx}"

                    # Preserve metadata and add chunk-specific info
                    chunk_metadata = content_data.get('metadata', {}).copy()
                    chunk_metadata.update({
                        'chunk_id': chunk_id,
                        'start_idx': start_idx,
                        'end_idx': end_idx,
                        'chunk_size': len(chunk_text),
                        'total_content_length': content_length
                    })

                    chunk_entry = {
                        'id': chunk_id,
                        'url': content_data.get('url', ''),
                        'title': content_data.get('title', ''),
                        'headings': content_data.get('headings', []),
                        'content': chunk_text,
                        'metadata': chunk_metadata
                    }

                    chunks.append(chunk_entry)

            # Move to next chunk with overlap
            start_idx = end_idx - self.overlap_size
            if start_idx >= content_length or self.overlap_size <= 0:
                start_idx = end_idx

            # Prevent infinite loop
            if start_idx <= 0:
                start_idx = end_idx

        logging.info(f"Chunking completed. Generated {len(chunks)} chunks from content")
        return chunks

    def _split_large_chunk(self, chunk_text: str, max_size: int) -> List[str]:
        """Split a large chunk into smaller chunks."""
        sub_chunks = []
        start = 0

        while start < len(chunk_text):
            end = start + max_size
            if end >= len(chunk_text):
                # Last chunk, just take the remainder
                sub_chunks.append(chunk_text[start:])
                break

            # Try to break at sentence or paragraph boundary
            sub_chunk = chunk_text[start:end]
            sentence_end = max(
                sub_chunk.rfind('. '),
                sub_chunk.rfind('! '),
                sub_chunk.rfind('? '),
                sub_chunk.rfind('\n'),
                sub_chunk.rfind('.\n'),
                sub_chunk.rfind('!'),
                sub_chunk.rfind('?')
            )

            if sentence_end > len(sub_chunk) // 2:  # Break at good boundary if it's in the second half
                actual_end = start + sentence_end + 1
                sub_chunks.append(chunk_text[start:actual_end])
                start = actual_end
            else:
                # No good boundary found, just split at max_size
                sub_chunks.append(chunk_text[start:end])
                start = end

        return sub_chunks

    def chunk_with_headings(self, content_data: Dict) -> List[Dict]:
        """Chunk content while preserving heading context."""
        content = content_data.get('content', '')
        headings = content_data.get('headings', [])

        logging.info(f"Starting chunking with headings for URL: {content_data.get('url', 'unknown')}")

        if not content:
            logging.warning(f"No content to chunk for URL: {content_data.get('url', 'unknown')}")
            return []

        # For now, use the basic chunking method
        # In the future, we could implement more sophisticated heading-aware chunking
        result = self.chunk_content(content_data)
        logging.info(f"Chunking with headings completed. Generated {len(result)} chunks.")
        return result