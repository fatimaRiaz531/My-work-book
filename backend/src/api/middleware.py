from fastapi import Request, HTTPException, status
from collections import defaultdict
from datetime import datetime, timedelta
import time
import logging

logger = logging.getLogger(__name__)

# Simple in-memory rate limiter (for demonstration)
# In production, use Redis or similar for distributed rate limiting
request_counts = defaultdict(list)


def rate_limit(max_requests: int = 100, window_seconds: int = 3600):
    """
    Rate limiting middleware decorator.

    Args:
        max_requests: Maximum number of requests allowed
        window_seconds: Time window in seconds
    """
    def rate_limit_middleware(request: Request):
        client_ip = request.client.host
        now = datetime.utcnow()
        window_start = now - timedelta(seconds=window_seconds)

        # Clean old requests outside the window
        request_counts[client_ip] = [
            req_time for req_time in request_counts[client_ip]
            if req_time > window_start
        ]

        # Check if limit exceeded
        if len(request_counts[client_ip]) >= max_requests:
            logger.warning(f"Rate limit exceeded for IP: {client_ip}")
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail="Rate limit exceeded. Please try again later."
            )

        # Add current request timestamp
        request_counts[client_ip].append(now)

    return rate_limit_middleware


# Authentication middleware
async def verify_api_key(request: Request):
    """
    Verify API key from header.
    In a real implementation, you might want to use OAuth or JWT tokens.
    """
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authorization header missing or invalid"
        )

    # In a real implementation, validate the token against a database or external service
    # For now, we'll just check that it's present
    api_key = auth_header[7:]  # Remove "Bearer " prefix

    if not api_key or len(api_key.strip()) == 0:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="API key is required"
        )

    # Log the request for audit purposes
    logger.info(f"Valid API key provided for endpoint: {request.url.path}")