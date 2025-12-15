from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Dict, Optional
import logging
from ..config.database import get_qdrant
from ..config.settings import settings
from ..utils.connection_validator import validate_all_connections
import openai

logger = logging.getLogger(__name__)

router = APIRouter()

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: Dict[str, str]
    details: Optional[Dict] = None


@router.get("/health", response_model=HealthResponse)
async def health_check(
    include_details: bool = Query(False, description="Include detailed connection information")
):
    """
    Health check endpoint to verify the service and its dependencies are running properly.

    Args:
        include_details: Whether to include detailed connection validation information
    """
    try:
        # Check OpenAI connection
        openai_status = "unhealthy"
        try:
            # Make a simple request to OpenAI API to verify it's working
            client = openai.OpenAI(api_key=settings.openai_api_key)
            client.models.list()
            openai_status = "healthy"
        except Exception as e:
            logger.error(f"OpenAI health check failed: {str(e)}")
            openai_status = "unhealthy"

        # Get comprehensive database connection status
        db_validation = validate_all_connections()

        # Extract individual statuses
        qdrant_status = db_validation["qdrant"]["status"]
        postgres_status = db_validation["postgres"]["status"]

        # Convert status to health format
        qdrant_health = "healthy" if qdrant_status == "connected" else "unhealthy"
        postgres_health = "healthy" if postgres_status == "connected" else "unhealthy"

        # Overall status
        overall_status = "healthy" if all([
            qdrant_health == "healthy",
            postgres_health == "healthy",
            openai_status == "healthy"
        ]) else "unhealthy"

        from datetime import datetime
        response = HealthResponse(
            status=overall_status,
            timestamp=datetime.utcnow().isoformat(),
            dependencies={
                "qdrant": qdrant_health,
                "postgres": postgres_health,
                "openai": openai_status
            }
        )

        # Include detailed validation info if requested
        if include_details:
            response.details = db_validation

        return response

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=503, detail="Service is unhealthy")