# RAG Chat API with OpenRouter Integration

## Overview
This system provides a RAG (Retrieval-Augmented Generation) chat API that integrates with:
- OpenRouter API for LLM inference
- Qdrant vector database for document retrieval
- FastAPI backend with CORS support for Docusaurus frontend

## Prerequisites

- Python 3.11+
- OpenRouter API key (get from https://openrouter.ai/keys)
- Qdrant cloud account with embeddings already stored

## Installation

1. Clone the repository
2. Navigate to the `backend` directory
3. Install dependencies:

```bash
pip install -r requirements.txt
```

## Configuration

Create a `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Then update the following values in your `.env` file:

### Environment Variables (.env file)
```env
# OpenRouter Configuration
OPENROUTER_API_KEY="your_openrouter_api_key_here"  # Get from https://openrouter.ai/keys
AGENT_MODEL="openai/gpt-3.5-turbo"  # or other models like "openai/gpt-4", "anthropic/claude-3-haiku", etc.
OPENROUTER_API_BASE="https://openrouter.ai/api/v1"

# Qdrant Configuration
QDRANT_URL="https://736aba90-996d-4dc4-b33e-766794a694c9.us-east4-0.gcp.cloud.qdrant.io:6333"
QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.GihVZ5EqDYQySLtVBKneqLqSyK_jgO0e9T0euD05zmg"
QDRANT_COLLECTION_NAME=book_embeddings
```

### Available Models on OpenRouter
Some popular models you can use:
- `openai/gpt-3.5-turbo`
- `openai/gpt-4`
- `anthropic/claude-3-haiku`
- `anthropic/claude-3-sonnet`
- `google/gemini-pro`
- And many more - see https://openrouter.ai/models

## Usage

### Running the API Server

Start the RAG chat API server:

```bash
python api.py
```

The server will be available at http://localhost:8000

## API Endpoints

### Health Check
- `GET /health` - Check if the service is running
- `GET /test` - Additional test endpoint

### Chat Endpoint
- `POST /chat` - Main chat endpoint for RAG queries

Request body:
```json
{
  "query": "Your question here",
  "user_context": {},
  "session_id": "optional session id",
  "metadata": {}
}
```

Response:
```json
{
  "response": "Generated response",
  "source_chunks": [
    {
      "id": "chunk_id",
      "content": "retrieved content",
      "relevance_score": 0.85,
      "source_document": "document_name"
    }
  ],
  "grounded": true,
  "query_id": "unique_query_id",
  "confidence": 0.89,
  "timestamp": "2023-12-28 15:30:45"
}
```

## Architecture

- **agent.py**: Contains the RAG agent that retrieves from Qdrant and generates responses via OpenRouter
- **api.py**: FastAPI server with endpoints and CORS configuration
- **Qdrant**: Vector database for document storage and similarity search
- **OpenRouter**: LLM inference service with support for multiple models

## Frontend Integration

The API is configured with CORS to work with your Docusaurus frontend. The chat endpoint returns the exact format expected by your frontend.

## Troubleshooting

1. **Connection errors**: Verify your Qdrant URL and API key are correct
2. **API key errors**: Make sure your OpenRouter API key is valid and has sufficient credits
3. **Embedding errors**: Ensure the embedding model matches your Qdrant collection
4. **CORS issues**: The server allows all origins for development; restrict in production