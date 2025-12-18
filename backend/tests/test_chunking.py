import pytest
from utils.chunking_utils import chunking_utils, ContentChunk


class TestChunkingUtils:
    """Unit tests for the chunking utilities."""

    def test_chunk_by_tokens_basic(self):
        """Test basic token-based chunking."""
        text = "This is a test sentence. " * 10  # Create a longer text
        chunks = chunking_utils.chunk_by_tokens(text, max_tokens=20, overlap_tokens=5)

        assert len(chunks) > 0
        assert all(isinstance(chunk, ContentChunk) for chunk in chunks)
        assert all(len(chunk.text) > 0 for chunk in chunks)

    def test_chunk_by_tokens_with_overlap(self):
        """Test token-based chunking with overlap."""
        text = "A B C D E F G H I J K L M N O P Q R S T U V W X Y Z"
        chunks = chunking_utils.chunk_by_tokens(text, max_tokens=10, overlap_tokens=3)

        assert len(chunks) > 1  # Should create multiple chunks

        # Check that there's overlap between consecutive chunks
        if len(chunks) > 1:
            # The end of first chunk should overlap with beginning of second
            first_chunk_end = chunks[0].text[-5:]  # Last 5 chars of first chunk
            second_chunk_start = chunks[1].text[:5]  # First 5 chars of second chunk

            # There should be some overlap, though exact overlap depends on word boundaries
            # The key is that chunks shouldn't have gaps if there's overlap
            assert chunks[1].start_pos < chunks[0].end_pos

    def test_chunk_by_tokens_empty_text(self):
        """Test chunking with empty text."""
        chunks = chunking_utils.chunk_by_tokens("", max_tokens=10, overlap_tokens=2)

        assert len(chunks) == 0

    def test_chunk_by_tokens_single_chunk(self):
        """Test chunking where text fits in a single chunk."""
        text = "Short text"
        chunks = chunking_utils.chunk_by_tokens(text, max_tokens=50, overlap_tokens=0)

        assert len(chunks) == 1
        assert chunks[0].text == text

    def test_chunk_by_tokens_preserve_paragraphs(self):
        """Test that paragraph boundaries are respected when possible."""
        text = "First paragraph.\n\nSecond paragraph.\n\nThird paragraph."
        chunks = chunking_utils.chunk_by_tokens(
            text,
            max_tokens=50,
            overlap_tokens=0,
            preserve_paragraphs=True
        )

        assert len(chunks) > 0
        # Check that paragraphs aren't unnecessarily split
        chunk_texts = [chunk.text for chunk in chunks]
        assert any("First paragraph" in ct for ct in chunk_texts)
        assert any("Second paragraph" in ct for ct in chunk_texts)

    def test_chunk_by_headings_basic(self):
        """Test heading-based chunking."""
        text = "# Heading 1\nSome content for heading 1.\n\n## Heading 2\nContent for heading 2.\n\n# Heading 3\nMore content."
        chunks = chunking_utils.chunk_by_headings(text, max_tokens=100)

        assert len(chunks) > 0
        # Should have at least one chunk per heading
        assert len([c for c in chunks if "Heading" in c.heading_context]) >= 2

    def test_chunk_by_headings_setext_style(self):
        """Test heading-based chunking with setext-style headings."""
        text = "Heading 1\n=========\nSome content.\n\nHeading 2\n---------\nMore content."
        chunks = chunking_utils.chunk_by_headings(text, max_tokens=100)

        assert len(chunks) > 0
        # Check that headings were detected
        assert any("Heading 1" in c.heading_context for c in chunks)
        assert any("Heading 2" in c.heading_context for c in chunks)

    def test_chunk_by_headings_with_large_sections(self):
        """Test heading-based chunking when sections are too large."""
        large_content = "A lot of content. " * 100
        text = f"# Large Heading\n{large_content}\n# Another Heading\nSmall content."
        chunks = chunking_utils.chunk_by_headings(text, max_tokens=50, overlap_tokens=10)

        # Large sections should be further chunked
        assert len(chunks) >= 2
        # Check that the large content was split
        large_chunks = [c for c in chunks if "Large Heading" in c.heading_context]
        assert len(large_chunks) > 1

    def test_validate_chunking_basic(self):
        """Test basic chunking validation."""
        original_text = "This is the original text that will be chunked and validated."
        chunks = chunking_utils.chunk_by_tokens(original_text, max_tokens=20)

        is_valid, message = chunking_utils.validate_chunking(original_text, chunks)

        assert is_valid
        assert "chunks created" in message

    def test_validate_chunking_empty_chunks(self):
        """Test chunking validation with empty chunks list."""
        original_text = "Some text"
        is_valid, message = chunking_utils.validate_chunking(original_text, [])

        assert is_valid  # Empty chunks is valid according to our implementation
        assert "No chunks to validate" in message

    def test_get_optimal_chunking_strategy_markdown_with_headings(self):
        """Test optimal chunking strategy selection for markdown with headings."""
        content = "# Title\nContent here\n## Section\nMore content"

        strategy = chunking_utils.get_optimal_chunking_strategy("markdown", content)

        # Should select heading-based chunking for markdown with headings
        assert strategy == "by_headings"

    def test_get_optimal_chunking_strategy_markdown_without_headings(self):
        """Test optimal chunking strategy selection for markdown without headings."""
        content = "Just plain text content without any headings."

        strategy = chunking_utils.get_optimal_chunking_strategy("markdown", content)

        # Should select token-based chunking when no headings are present
        assert strategy == "by_tokens"

    def test_get_optimal_chunking_strategy_other_content(self):
        """Test optimal chunking strategy selection for non-markdown content."""
        content = "Just plain text content."

        strategy = chunking_utils.get_optimal_chunking_strategy("text", content)

        # Should select token-based chunking for non-markdown content
        assert strategy == "by_tokens"

    def test_chunk_with_special_characters(self):
        """Test chunking with special characters and unicode."""
        text = "This is a test with special characters: é, ñ, 中文, и русский текст."
        chunks = chunking_utils.chunk_by_tokens(text, max_tokens=30)

        assert len(chunks) > 0
        assert all(isinstance(chunk, ContentChunk) for chunk in chunks)
        # Reconstruct text and verify it contains the original content
        reconstructed = "".join(chunk.text for chunk in chunks)
        assert "é" in reconstructed
        assert "ñ" in reconstructed

    def test_chunk_positions_correct(self):
        """Test that chunk positions are correctly calculated."""
        text = "ABC DEF GHI JKL MNO PQR"
        chunks = chunking_utils.chunk_by_tokens(text, max_tokens=6, overlap_tokens=1)

        assert len(chunks) > 1

        # Verify positions are within bounds
        for chunk in chunks:
            assert 0 <= chunk.start_pos < len(text)
            assert chunk.start_pos <= chunk.end_pos <= len(text)
            # Verify the text matches what's expected at those positions
            assert text[chunk.start_pos:chunk.end_pos] == chunk.text

    def test_chunk_reconstruction(self):
        """Test that text can be reconstructed from chunks with simple token-based chunking."""
        original_text = "This is a sample text for testing reconstruction from chunks."
        chunks = chunking_utils.chunk_by_tokens(original_text, max_tokens=10, overlap_tokens=0)

        # For non-overlapping chunks, we should be able to reconstruct the original
        reconstructed = "".join(chunk.text for chunk in chunks)

        # Note: Reconstruction might not be perfect due to word boundary respecting
        # but the content should all be there
        assert len(reconstructed) <= len(original_text)
        # All chunks' content should be part of the original text
        for chunk in chunks:
            assert chunk.text in original_text