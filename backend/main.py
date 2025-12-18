from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from api.health import router as health_router
from api.ingest_router import router as ingest_router
from api.query_router import router as query_router
from config import settings


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(
        title="RAG Chatbot API",
        description="API for the Integrated RAG Chatbot for Docusaurus Technical Book",
        version="1.0.0",
        debug=settings.app_debug,
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["http://localhost:3000"],  # In production, replace with specific origins
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include routers
    app.include_router(health_router, prefix="/health", tags=["health"])
    app.include_router(ingest_router, prefix="/api", tags=["ingestion"])
    app.include_router(query_router, prefix="/api", tags=["query"])

    return app


app = create_app()


@app.on_event("startup")
async def startup_event():
    """Handle startup events."""
    print("Application starting up...")


@app.on_event("shutdown")
async def shutdown_event():
    """Handle shutdown events."""
    print("Application shutting down...")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host=settings.app_host,
        port=settings.app_port,
        reload=settings.app_env == "development",
    )