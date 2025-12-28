# Data Model: FastAPI RAG Integration

## QueryRequest Entity
Represents a user query with optional metadata, containing fields like query text, user context, and session information.

**Fields:**
- `query` (string, required): The user's question or query text
- `user_context` (object, optional): Additional context about the user or session
- `session_id` (string, optional): Identifier for the conversation session
- `metadata` (object, optional): Additional metadata for the query

## QueryResponse Entity
Represents the RAG-generated response with answer text, source document references, confidence scores, and processing metadata.

**Fields:**
- `response` (string, required): The generated answer/response text
- `source_chunks` (array, optional): Array of source document chunks used to generate the response
- `grounded` (boolean, required): Whether the response is grounded in retrieved context
- `query_id` (string, required): Identifier for the query session
- `confidence` (number, required): Confidence score for the response (0.0-1.0)
- `timestamp` (string, required): ISO timestamp of the response generation

## SourceChunk Entity (nested in QueryResponse)
Represents a chunk of source document used in the response generation.

**Fields:**
- `id` (string, required): Unique identifier for the document chunk
- `content` (string, required): The content of the document chunk
- `relevance_score` (number, required): Relevance score for this chunk (0.0-1.0)
- `source_document` (string, required): Reference to the source document

## API Contract: /chat Endpoint
**Method**: POST
**Path**: `/chat`
**Request Body**: QueryRequest object
**Response**: QueryResponse object
**Status Codes**:
- 200: Success with response
- 400: Invalid request format
- 500: Internal server error
- 503: RAG backend unavailable