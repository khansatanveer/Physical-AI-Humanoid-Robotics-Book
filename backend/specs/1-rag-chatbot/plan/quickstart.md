# Quickstart Guide: Integrated RAG Chatbot

## Prerequisites

- Python 3.9+ installed
- Node.js 16+ installed (for Docusaurus)
- Access to Cohere API (API key)
- Qdrant Cloud account (API key and URL)
- Neon Serverless Postgres account (connection string)

## Environment Setup

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Create environment file:
```bash
cp .env.example .env
```

3. Update `.env` with your credentials:
```env
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
NEON_DATABASE_URL=your_neon_postgres_connection_string
```

## Backend Setup

1. Navigate to backend directory:
```bash
cd backend
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Start the FastAPI server:
```bash
uvicorn main:app --reload --port 8000
```

The backend API will be available at `http://localhost:8000`.

## Frontend Setup

1. Navigate to frontend directory:
```bash
cd frontend  # or wherever Docusaurus is located
```

2. Install dependencies:
```bash
npm install
```

3. Start the development server:
```bash
npm start
```

The Docusaurus site with the integrated chatbot will be available at `http://localhost:3000`.

## Initial Content Ingestion

1. Prepare your book content in the required format
2. Make a POST request to the `/ingest` endpoint:
```bash
curl -X POST http://localhost:8000/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Your book content here...",
    "metadata": {
      "module": "Module Name",
      "chapter": "Chapter Name",
      "section": "Section Name",
      "heading_path": "Module > Chapter > Section",
      "source_url": "/docs/module/chapter/section"
    }
  }'
```

## Using the Chatbot

### Global Query Mode
Ask questions about the entire book content:
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "Your question here"}'
```

### Selected Text Mode
Ask questions about specific selected text:
```bash
curl -X POST http://localhost:8000/query/selection \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Your question about the text",
    "selected_text": "The specific text selected by the user..."
  }'
```

## Verification

1. Check that all services are running:
   - Backend: `http://localhost:8000/health`
   - Frontend: `http://localhost:3000`

2. Test the chatbot functionality by asking a question about your book content

3. Verify that responses include proper source attribution

## Troubleshooting

- If ingestion fails, check that your Qdrant credentials are correct
- If responses are slow, verify your Cohere API access
- If the chatbot UI doesn't appear, ensure the React component is properly integrated into Docusaurus