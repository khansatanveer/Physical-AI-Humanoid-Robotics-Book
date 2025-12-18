# Data Model: Integrated RAG Chatbot

**Feature**: 1-rag-chatbot
**Date**: 2025-12-18
**Status**: Draft

## Entity Definitions

### BookContent
Represents indexed book content with metadata for RAG retrieval.

**Fields**:
- `id` (string): Unique identifier for the content chunk
- `content` (string): The actual text content of the chunk
- `module` (string): The module name this content belongs to
- `chapter` (string): The chapter name within the module
- `section` (string): The section name within the chapter
- `heading_path` (string): Full heading hierarchy (e.g., "Module 1 > Chapter 2 > Section 3")
- `source_url` (string): URL/path to the original content location
- `embedding` (vector): Vector representation for similarity search
- `created_at` (timestamp): When this content was indexed
- `updated_at` (timestamp): When this content was last updated

**Validation Rules**:
- All metadata fields required
- Content length between 10 and 10000 characters
- Embedding must be a valid vector representation

### ChatInteraction
Represents a user's question and the system's response.

**Fields**:
- `id` (string): Unique identifier for the interaction
- `user_id` (string): Identifier for the user (optional for anonymous)
- `query` (string): The user's original question
- `response` (string): The system's generated response
- `query_type` (enum): Type of query ("global" or "selection")
- `selected_text` (string): Text selected by user (for selection queries only)
- `retrieved_chunks` (array): IDs of content chunks used to generate response
- `confidence_score` (number): Confidence level in the response (0-1)
- `source_attribution` (array): Source references for the response
- `timestamp` (timestamp): When the interaction occurred
- `session_id` (string): Session identifier for conversation history

**Validation Rules**:
- Query must be between 1 and 1000 characters
- Query type must be either "global" or "selection"
- Response must not exceed 5000 characters

### QueryLog
Represents logged queries for analytics and debugging.

**Fields**:
- `id` (string): Unique identifier for the log entry
- `query` (string): The original query
- `response_status` (enum): Status of the response ("success", "error", "no_content")
- `retrieval_time` (number): Time taken for content retrieval (ms)
- `generation_time` (number): Time taken for response generation (ms)
- `total_time` (number): Total processing time (ms)
- `retrieved_count` (number): Number of content chunks retrieved
- `user_agent` (string): Client information (optional)
- `ip_address` (string): User IP for analytics (respecting privacy)
- `timestamp` (timestamp): When the query was processed

**Validation Rules**:
- Query must be between 1 and 1000 characters
- Response status must be one of the defined enum values

### UserFeedback
Represents optional feedback provided by users about response quality.

**Fields**:
- `id` (string): Unique identifier for the feedback
- `interaction_id` (string): Reference to the chat interaction
- `rating` (number): Numerical rating (1-5 scale)
- `thumbs` (enum): Like/dislike ("up", "down", null)
- `comment` (string): Optional textual feedback
- `useful` (boolean): Whether the response was helpful
- `accuracy_rating` (number): Accuracy rating (1-5 scale)
- `timestamp` (timestamp): When feedback was provided

**Validation Rules**:
- Rating must be between 1 and 5
- Thumbs must be "up", "down", or null
- Comment length must be under 1000 characters