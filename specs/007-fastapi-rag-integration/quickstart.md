# Quickstart: FastAPI RAG Integration

## Prerequisites
- Python 3.8+
- FastAPI and related dependencies
- Existing agent system from `agent.py`
- Docusaurus documentation setup

## Setup Instructions

### 1. Install Dependencies
```bash
pip install fastapi uvicorn python-dotenv
```

### 2. Create the API Server (`api.py`)
```python
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import asyncio
import os
from agent import RetrievalAgent

app = FastAPI(title="RAG Chat API", version="1.0.0")

# Initialize the retrieval agent
agent = RetrievalAgent()

class QueryRequest(BaseModel):
    query: str
    user_context: Optional[Dict[str, Any]] = None
    session_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None

class SourceChunk(BaseModel):
    id: str
    content: str
    relevance_score: float
    source_document: str

class QueryResponse(BaseModel):
    response: str
    source_chunks: Optional[List[SourceChunk]] = None
    grounded: bool
    query_id: str
    confidence: float
    timestamp: str

@app.post("/chat", response_model=QueryResponse)
async def chat_endpoint(request: QueryRequest):
    try:
        # Process the query using the retrieval agent
        result = await agent.query(request.query)
        return QueryResponse(
            response=result["response"],
            source_chunks=result.get("source_chunks"),
            grounded=result["grounded"],
            query_id=result["query_id"],
            confidence=result["confidence"],
            timestamp=result.get("timestamp", "")
        )
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"RAG backend unavailable: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

### 3. Run the API Server
```bash
python api.py
```

### 4. Test the API
```bash
curl -X POST "http://localhost:8000/chat" \
  -H "Content-Type: application/json" \
  -d '{"query": "What are the key concepts in humanoid robotics?"}'
```

## Frontend Integration
The Docusaurus-based UI in `Physical-AI-Humanoid-Robotics-book` can call the `/chat` endpoint to integrate with the RAG system.

## API Documentation
Once running, the API documentation will be available at `http://localhost:8000/docs`