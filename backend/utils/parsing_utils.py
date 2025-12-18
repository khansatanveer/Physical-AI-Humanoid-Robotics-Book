import re
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from pathlib import Path


@dataclass
class ParsedContent:
    """Represents parsed content with metadata."""
    title: str
    content: str
    headings: List[Dict[str, str]]  # List of {level: str, text: str}
    metadata: Dict[str, str]
    file_path: Optional[str] = None


class ParsingUtils:
    """Utility class for parsing different content formats."""

    @staticmethod
    def parse_markdown(
        content: str,
        file_path: Optional[str] = None
    ) -> ParsedContent:
        """
        Parse Markdown content and extract metadata and headings.

        Args:
            content: The Markdown content to parse
            file_path: Optional file path for reference

        Returns:
            ParsedContent object with extracted information
        """
        # Extract title (first heading or first non-empty line)
        title = ParsingUtils._extract_title_from_markdown(content)

        # Extract headings
        headings = ParsingUtils._extract_headings_from_markdown(content)

        # Extract frontmatter metadata if present
        metadata = ParsingUtils._extract_frontmatter(content)

        return ParsedContent(
            title=title,
            content=content,
            headings=headings,
            metadata=metadata,
            file_path=file_path
        )

    @staticmethod
    def _extract_title_from_markdown(content: str) -> str:
        """Extract title from Markdown content."""
        lines = content.strip().split('\n')

        # Look for first heading
        for line in lines:
            heading_match = re.match(r'^(#{1,6})\s+(.+)$', line)
            if heading_match:
                return heading_match.group(2).strip()

        # If no heading found, use first non-empty line
        for line in lines:
            stripped = line.strip()
            if stripped and not stripped.startswith('<!--'):
                return stripped[:100]  # Limit title length

        return "Untitled"

    @staticmethod
    def _extract_headings_from_markdown(content: str) -> List[Dict[str, str]]:
        """Extract all headings from Markdown content."""
        headings = []

        # Match ATX headings (e.g., # Heading 1, ## Heading 2)
        atx_pattern = r'^(#{1,6})\s+(.+)$'
        lines = content.split('\n')

        for line in lines:
            atx_match = re.match(atx_pattern, line)
            if atx_match:
                level = len(atx_match.group(1))
                text = atx_match.group(2).strip()
                headings.append({"level": str(level), "text": text})

        # Match Setext headings (underlined headings)
        for i in range(len(lines) - 1):
            current_line = lines[i].strip()
            next_line = lines[i + 1].strip()

            if current_line and not current_line.startswith('#') and next_line:
                if re.match(r'^=+$', next_line):  # H1
                    headings.append({"level": "1", "text": current_line})
                elif re.match(r'^-+$', next_line):  # H2
                    headings.append({"level": "2", "text": current_line})

        return headings

    @staticmethod
    def _extract_frontmatter(content: str) -> Dict[str, str]:
        """Extract frontmatter metadata from content if present."""
        metadata = {}

        # Look for YAML frontmatter (enclosed between ---)
        frontmatter_pattern = r'^---\n(.*?)\n---'
        frontmatter_match = re.search(frontmatter_pattern, content, re.DOTALL | re.MULTILINE)

        if frontmatter_match:
            frontmatter_content = frontmatter_match.group(1)
            lines = frontmatter_content.split('\n')

            for line in lines:
                if ':' in line:
                    key, value = line.split(':', 1)
                    metadata[key.strip()] = value.strip().strip('"\'')  # Remove quotes

        return metadata

    @staticmethod
    def parse_docusaurus_doc(
        content: str,
        file_path: Optional[str] = None
    ) -> ParsedContent:
        """
        Parse Docusaurus documentation content with special handling.

        Args:
            content: The Docusaurus doc content to parse
            file_path: Optional file path for reference

        Returns:
            ParsedContent object with extracted information
        """
        # Parse as regular markdown first
        parsed = ParsingUtils.parse_markdown(content, file_path)

        # Extract Docusaurus-specific metadata
        docusaurus_metadata = ParsingUtils._extract_docusaurus_metadata(content)

        # Update metadata with Docusaurus-specific fields
        parsed.metadata.update(docusaurus_metadata)

        return parsed

    @staticmethod
    def _extract_docusaurus_metadata(content: str) -> Dict[str, str]:
        """Extract Docusaurus-specific metadata from content."""
        metadata = {}

        # Extract Docusaurus-specific frontmatter fields
        # Common Docusaurus fields: id, title, sidebar_label, description, keywords
        docusaurus_fields = [
            'id', 'title', 'sidebar_label', 'description', 'keywords',
            'slug', 'tags', 'authors', 'image', 'draft'
        ]

        # Look for frontmatter
        frontmatter_pattern = r'^---\n(.*?)\n---'
        frontmatter_match = re.search(frontmatter_pattern, content, re.DOTALL | re.MULTILINE)

        if frontmatter_match:
            frontmatter_content = frontmatter_match.group(1)
            lines = frontmatter_content.split('\n')

            for line in lines:
                line = line.strip()
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip().strip('"\'')  # Remove quotes

                    if key in docusaurus_fields:
                        metadata[key] = value

        return metadata

    @staticmethod
    def parse_text(
        content: str,
        file_path: Optional[str] = None
    ) -> ParsedContent:
        """
        Parse plain text content.

        Args:
            content: The plain text content to parse
            file_path: Optional file path for reference

        Returns:
            ParsedContent object with extracted information
        """
        # For plain text, use first line as title
        lines = content.strip().split('\n')
        title = lines[0][:100] if lines and lines[0].strip() else "Untitled"

        return ParsedContent(
            title=title,
            content=content,
            headings=[],
            metadata={},
            file_path=file_path
        )

    @staticmethod
    def parse_content(
        content: str,
        content_type: str = "markdown",
        file_path: Optional[str] = None
    ) -> ParsedContent:
        """
        Parse content based on its type.

        Args:
            content: The content to parse
            content_type: Type of content ("markdown", "docusaurus", "text")
            file_path: Optional file path for reference

        Returns:
            ParsedContent object with extracted information
        """
        if content_type.lower() == "docusaurus":
            return ParsingUtils.parse_docusaurus_doc(content, file_path)
        elif content_type.lower() in ["markdown", "md", "mdx"]:
            return ParsingUtils.parse_markdown(content, file_path)
        else:
            return ParsingUtils.parse_text(content, file_path)

    @staticmethod
    def extract_module_chapter_section_from_path(
        file_path: str
    ) -> Tuple[str, str, str, str]:
        """
        Extract module, chapter, and section from file path.

        Args:
            file_path: Path to the content file

        Returns:
            Tuple of (module, chapter, section, heading_path)
        """
        path = Path(file_path)
        parts = path.parts

        # Remove common prefixes like 'docs', 'src', etc.
        common_prefixes = ['docs', 'src', 'content', 'pages']
        clean_parts = []
        found_content = False

        for part in parts:
            if not found_content and part in common_prefixes:
                continue
            else:
                found_content = True
                clean_parts.append(part.replace('_', ' ').replace('-', ' ').title())

        # Remove file extension from the last part
        if clean_parts:
            last_part = clean_parts[-1]
            if '.' in last_part:
                clean_parts[-1] = last_part.rsplit('.', 1)[0]

        # Determine module, chapter, section based on path depth
        if len(clean_parts) >= 3:
            module = clean_parts[0]
            chapter = clean_parts[1]
            section = clean_parts[2] if len(clean_parts) > 2 else clean_parts[-1]
        elif len(clean_parts) == 2:
            module = clean_parts[0]
            chapter = clean_parts[1]
            section = clean_parts[1]  # Use chapter as section if only 2 levels
        elif len(clean_parts) == 1:
            module = "General"
            chapter = clean_parts[0]
            section = clean_parts[0]
        else:
            module = "General"
            chapter = "Overview"
            section = "Introduction"

        # Create heading path
        heading_path = " > ".join(clean_parts) if clean_parts else "General > Overview > Introduction"

        return module, chapter, section, heading_path

    @staticmethod
    def clean_content(content: str) -> str:
        """
        Clean content by removing unnecessary elements.

        Args:
            content: The content to clean

        Returns:
            Cleaned content string
        """
        # Remove HTML comments
        content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

        # Remove extra whitespace while preserving structure
        lines = content.split('\n')
        cleaned_lines = []

        for line in lines:
            # Remove trailing whitespace
            line = line.rstrip()

            # Skip empty lines that are just whitespace
            if line or cleaned_lines:  # Keep content lines and preserve structure
                cleaned_lines.append(line)

        # Join lines and normalize whitespace
        cleaned_content = '\n'.join(cleaned_lines)
        cleaned_content = re.sub(r'\n\s*\n', '\n\n', cleaned_content)  # Remove excessive empty lines

        return cleaned_content.strip()

    @staticmethod
    def extract_code_blocks(content: str) -> List[Dict[str, str]]:
        """
        Extract code blocks from content.

        Args:
            content: The content to extract code blocks from

        Returns:
            List of code blocks with language and content
        """
        code_blocks = []
        pattern = r'```(\w*)\n(.*?)```'

        for match in re.finditer(pattern, content, re.DOTALL):
            language = match.group(1).strip() if match.group(1) else 'text'
            code = match.group(2).rstrip('\n')  # Remove trailing newlines
            code_blocks.append({
                'language': language,
                'code': code
            })

        return code_blocks

    @staticmethod
    def extract_links(content: str) -> List[Dict[str, str]]:
        """
        Extract links from content.

        Args:
            content: The content to extract links from

        Returns:
            List of links with text and URL
        """
        links = []

        # Match Markdown links: [text](url)
        markdown_pattern = r'\[([^\]]+)\]\(([^)]+)\)'
        for match in re.finditer(markdown_pattern, content):
            links.append({
                'text': match.group(1),
                'url': match.group(2)
            })

        # Match bare URLs
        url_pattern = r'https?://[^\s<>"\']+'
        for match in re.finditer(url_pattern, content):
            # Check if this URL is already captured in a Markdown link
            is_duplicate = any(match.group(0) == link['url'] for link in links)
            if not is_duplicate:
                links.append({
                    'text': match.group(0),
                    'url': match.group(0)
                })

        return links


# Convenience instance
parsing_utils = ParsingUtils()