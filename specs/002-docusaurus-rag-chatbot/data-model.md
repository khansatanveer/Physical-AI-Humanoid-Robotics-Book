# Data Model: Docusaurus Embedded RAG Chatbot UI

## Core Entities

### Query Request (Frontend State)
- **Fields**:
  - `query_text` (string): The user's question
  - `selected_text` (string, optional): Text selected by user (for selected-text mode)
  - `query_mode` (enum): "global" or "selection"
  - `timestamp` (datetime): When the query was created

### Query Response (API Response)
- **Fields**:
  - `answer` (string): The generated answer
  - `sources` (array of objects): List of source documents/chunks used
    - `document_id` (string): Unique identifier for the source
    - `section_title` (string): Title of the section
    - `content_snippet` (string): Relevant content snippet
    - `similarity_score` (float): Similarity score for the match (optional)
  - `query_id` (string): Unique identifier for the query
  - `timestamp` (datetime): When the response was generated

### Chat Message (Frontend State)
- **Fields**:
  - `id` (string): Unique identifier for the message
  - `role` (enum): "user" or "assistant"
  - `content` (string): The message content
  - `sources` (array of objects, optional): Sources for assistant responses
  - `timestamp` (datetime): When the message was created
  - `status` (enum): "pending", "success", or "error"

### Chat Session (Frontend State)
- **Fields**:
  - `id` (string): Unique identifier for the session
  - `messages` (array of ChatMessage): List of messages in the session
  - `currentMode` (enum): "global" or "selection"
  - `createdAt` (datetime): When the session was created

## API Request/Response Models

### Global Query Request
- **Fields**:
  - `query` (string): The user's question (required)
  - `user_id` (string, optional): User identifier for tracking
  - `session_id` (string, optional): Session identifier

### Selection Query Request
- **Fields**:
  - `query` (string): The user's question (required)
  - `selected_text` (string): The selected text context (required for selection mode)
  - `user_id` (string, optional): User identifier for tracking
  - `session_id` (string, optional): Session identifier

### Query Response (from backend API)
- **Fields**:
  - `query_id` (string): Unique identifier for this query
  - `answer` (string): The AI-generated answer to the question
  - `sources` (array of objects): List of sources used to generate the answer
  - `timestamp` (string): When the response was generated (ISO 8601)
  - `processing_time` (number, optional): Time taken to process the query in seconds

## Frontend State Management

### Chat State Structure
- `currentSession` (ChatSession): The active chat session
- `isLoading` (boolean): Whether a query is currently being processed
- `error` (string, optional): Error message if a request failed
- `selectedText` (string, optional): Currently selected text on the page
- `apiConfig` (object): Configuration for API communication
  - `baseUrl` (string): Base URL for the backend API
  - `timeout` (number): Request timeout in milliseconds

## Validation Rules (Frontend)

### Query Request Validation
- `query_text` must be 1-2000 characters
- `query_mode` must be either "global" or "selection"
- If `query_mode` is "selection", `selected_text` must be provided and not exceed 5000 characters
- Query must not be empty or only whitespace

### Message Validation
- Each message must have content
- User messages should be created with "pending" status initially
- Assistant messages should include source information when available

### Session Validation
- Sessions must have unique IDs
- Message timestamps should be in chronological order
- Session mode should be clearly indicated to the user