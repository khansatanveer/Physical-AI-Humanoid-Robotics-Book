# Data Model: Retrieval Pipeline for Book Content

## Overview
Data model for the retrieval pipeline that enables semantic similarity search against embedded book content in Qdrant.

## Key Entities

### Query
- **Description**: Text input from user that needs to be semantically matched against stored content
- **Fields**:
  - `text` (string): The query text input from the user
  - `embedding` (list[float]): Vector representation of the query text
  - `timestamp` (datetime): When the query was processed
  - `top_k` (int): Number of top results to retrieve (default: 5)

### RetrievedChunk
- **Description**: Content segment returned from the vector database that matches the query, containing text content and metadata
- **Fields**:
  - `id` (string): Unique identifier for the chunk
  - `content` (string): The actual text content of the chunk
  - `url` (string): URL where the content was sourced from
  - `title` (string): Title of the page/chapter where content appears
  - `headings` (list[string]): Headings associated with this content chunk
  - `score` (float): Similarity score between query and this chunk (0.0-1.0)
  - `metadata` (dict): Additional metadata about the content

### SimilarityScore
- **Description**: Numerical value representing the semantic similarity between the query and retrieved content
- **Fields**:
  - `value` (float): The similarity score (0.0-1.0, where 1.0 is most similar)
  - `algorithm` (string): The algorithm used to calculate the similarity (e.g., "cosine")

### Metadata
- **Description**: Associated information with each retrieved chunk including URL, headings, and content identifiers
- **Fields**:
  - `url` (string): Source URL of the content
  - `content_hash` (string): Hash of the content for duplicate detection
  - `created_at` (datetime): When this content was indexed
  - `headings` (list[string]): Headings associated with the content
  - `title` (string): Title of the source document

## Relationships
- A single `Query` can result in multiple `RetrievedChunk` objects
- Each `RetrievedChunk` has one `SimilarityScore`
- Each `RetrievedChunk` has one `Metadata` object
- Multiple `RetrievedChunk` objects can come from the same source `url`

## Validation Rules
- Query text must not be empty or null
- Query top_k must be a positive integer (1-100)
- RetrievedChunk content must not be empty
- SimilarityScore value must be between 0.0 and 1.0
- URL in metadata must be a valid URL format

## State Transitions
- Query: INITIALIZED → EMBEDDING_GENERATED → SEARCH_PERFORMED → RESULTS_RETURNED
- RetrievedChunk: CREATED → VALIDATED → FORMATTED → RETURNED