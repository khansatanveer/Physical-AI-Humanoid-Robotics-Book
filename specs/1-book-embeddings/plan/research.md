# Research: Book Website Embeddings Pipeline

## Decision: Web Crawling Approach
**Rationale**: For crawling Docusaurus-hosted book websites, we'll use `requests` + `beautifulsoup4` for basic crawling with `scrapy` as an alternative for more complex scenarios. This approach is lightweight and sufficient for most documentation sites.
**Alternatives considered**:
- Selenium (heavyweight, requires browser)
- Playwright (also heavyweight for simple crawling)
- requests + beautifulsoup4 (lightweight, efficient for static content)

## Decision: Content Chunking Strategy
**Rationale**: Using recursive character text splitting with hierarchical section awareness to maintain context while ensuring consistent chunk sizes. This preserves document structure and heading relationships.
**Alternatives considered**:
- Fixed-length splitting (breaks context)
- Semantic splitting (requires more complex NLP)
- Recursive character splitting (maintains context, simple implementation)

## Decision: Embedding Model Selection
**Rationale**: Cohere's embed-multilingual-v3.0 model chosen for its strong performance on technical documentation and multilingual support, which is important for book content.
**Alternatives considered**:
- OpenAI text-embedding-ada-002 (costlier, less optimized for technical content)
- Sentence Transformers (self-hosted, but less consistent quality)
- Cohere embed-english-v3.0 (limited to English)

## Decision: Qdrant Collection Schema
**Rationale**: Designing a schema that supports efficient similarity search with rich metadata for filtering and retrieval. Includes URL, headings, and content structure information.
**Alternatives considered**:
- Minimal metadata (reduces search capabilities)
- Complex nested structures (increases complexity without clear benefit)
- Simple flat structure (sufficient for our use case)

## Decision: Idempotent Processing Strategy
**Rationale**: Using URL + content hash as unique identifier to prevent duplicate processing. This ensures re-runs don't create duplicates while handling content updates.
**Alternatives considered**:
- Timestamp-based deduplication (less reliable)
- Content-only hashing (doesn't account for URL changes)
- URL-only identification (doesn't handle content updates)