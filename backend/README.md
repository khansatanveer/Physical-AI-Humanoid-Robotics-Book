# RAG Chatbot Backend

This is the backend service for the Integrated RAG Chatbot for Docusaurus Technical Book. It provides API endpoints for content ingestion, global RAG queries, and selected-text queries using Cohere AI models and Qdrant vector database.

## Features

- **Content Ingestion**: Index book content with metadata for RAG retrieval
- **Global RAG Queries**: Ask questions about the entire book content
- **Selected-Text Queries**: Ask questions about user-selected text only
- **Zero Hallucination**: Answers are strictly grounded in provided content
- **Source Attribution**: Responses include references to source sections

## Tech Stack

- **Backend Framework**: FastAPI
- **AI Provider**: Cohere (embeddings and text generation)
- **Vector Database**: Qdrant Cloud
- **Relational Database**: Neon Serverless Postgres
- **Language**: Python 3.11

## Setup

1. Clone the repository
2. Navigate to the backend directory: `cd backend`
3. Install dependencies: `pip install -r requirements.txt`
4. Create environment file: `cp .env.example .env`
5. Update `.env` with your API keys and service URLs
6. Start the development server: `uvicorn main:app --reload`

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_URL`: Your Qdrant cluster URL
- `NEON_DATABASE_URL`: Your Neon Postgres connection string

## API Endpoints

- `POST /ingest` - Index book content with metadata
- `POST /query` - Query book content with global knowledge
- `POST /query/selection` - Query using selected text only

## Development

- Code formatting: `black .`
- Import sorting: `isort .`
- Linting: `flake8 .`
- Type checking: `mypy .`
- Tests: `pytest`

## Architecture

The backend follows a clean architecture with separation of concerns:

- **API Layer**: FastAPI routers and request/response schemas
- **Service Layer**: Business logic and coordination between components
- **Client Layer**: External service integrations (Cohere, Qdrant, Postgres)
- **Model Layer**: Data models and validation
- **Utility Layer**: Helper functions and utilities