from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import asyncio
import os
from agent import RetrievalAgent
import logging


# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="RAG Chat API", version="1.0.0")

# Add CORS middleware to allow requests from Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local Docusaurus
        "http://localhost:8000",  # Local API
        "http://localhost:3001",  # Alternative local port
        "https://physical-ai-humanoid-robotics-book-orpin-sigma.vercel.app",  # Vercel deployment
        "https://physical-ai-humanoid-robotics-book.vercel.app",  # Alternative Vercel URL
        "https://*.vercel.app",  # Wildcard for Vercel previews
        "https://khansatanveer-deploy-chatbot.hf.space",  # Hugging Face Space
        "https://*.hf.space",  # Wildcard for Hugging Face Spaces
        "https://huggingface.co"  # Hugging Face main domain
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize the retrieval agent
try:
    agent = RetrievalAgent()
    logger.info("Retrieval agent initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize retrieval agent: {e}")
    raise


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
    """
    Chat endpoint that processes user queries using the RAG agent.

    Expects a JSON payload with a 'query' field.
    Returns a structured response with the answer and source information.
    """
    try:
        logger.info(f"Processing chat query: {request.query}")

        # Process the query using the retrieval agent
        result = agent.query(request.query)

        # Ensure source_chunks is properly formatted for the response
        source_chunks = []
        if result.get("source_chunks"):
            for chunk in result["source_chunks"]:
                source_chunks.append(SourceChunk(
                    id=chunk.get("id", ""),
                    content=chunk.get("content", ""),
                    relevance_score=chunk.get("relevance_score", 0.0),
                    source_document=chunk.get("source_document", "")
                ))

        return QueryResponse(
            response=result["response"],
            source_chunks=source_chunks if source_chunks else None,
            grounded=result["grounded"],
            query_id=result["query_id"],
            confidence=result["confidence"],
            timestamp=result.get("timestamp", "")
        )
    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error processing chat query: {e}")
        raise HTTPException(status_code=503, detail=f"RAG backend unavailable: {str(e)}")


@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "rag-chat-api"}


# Additional endpoint for testing connection
@app.get("/test")
async def test_connection():
    """
    Test endpoint to verify the API is working
    """
    return {"message": "RAG Chat API is running", "status": "ok"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)