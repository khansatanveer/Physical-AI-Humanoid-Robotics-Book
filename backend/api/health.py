from fastapi import APIRouter
from pydantic import BaseModel
from typing import Dict, Any

from clients.qdrant_client import qdrant_service
from clients.postgres_client import postgres_service


router = APIRouter()


class HealthStatus(BaseModel):
    """Health check response model."""
    status: str
    details: Dict[str, Any]


@router.get("/", response_model=HealthStatus)
async def health_check():
    """Health check endpoint to verify service availability."""
    qdrant_healthy = await qdrant_service.health_check()
    postgres_healthy = postgres_service.health_check()

    overall_status = "healthy" if qdrant_healthy and postgres_healthy else "unhealthy"

    details = {
        "qdrant": "healthy" if qdrant_healthy else "unhealthy",
        "postgres": "healthy" if postgres_healthy else "unhealthy",
        "timestamp": __import__('datetime').datetime.utcnow().isoformat()
    }

    return HealthStatus(status=overall_status, details=details)


@router.get("/ready", response_model=HealthStatus)
async def readiness_check():
    """Readiness check endpoint to verify service readiness."""
    # For readiness, we check if all required services are available
    qdrant_healthy = await qdrant_service.health_check()
    postgres_healthy = postgres_service.health_check()

    # Initialize Qdrant collection if needed
    try:
        await qdrant_service.initialize_collection()
        qdrant_ready = True
    except Exception:
        qdrant_ready = False

    overall_status = "ready" if qdrant_healthy and postgres_healthy and qdrant_ready else "not ready"

    details = {
        "qdrant": "ready" if qdrant_healthy and qdrant_ready else "not ready",
        "postgres": "ready" if postgres_healthy else "not ready",
        "timestamp": __import__('datetime').datetime.utcnow().isoformat()
    }

    return HealthStatus(status=overall_status, details=details)


@router.get("/live", response_model=HealthStatus)
async def liveness_check():
    """Liveness check endpoint to verify service is running."""
    # For liveness, we just check if the service itself is running
    # This is usually just returning healthy unless there's a critical failure
    return HealthStatus(
        status="alive",
        details={
            "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
            "service": "rag-chatbot-backend"
        }
    )