from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Union
import logging

logger = logging.getLogger(__name__)


async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions"""
    logger.error(f"HTTP Exception: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail}
    )


async def validation_exception_handler(request: Request, exc: Exception):
    """Handle validation and other exceptions"""
    logger.error(f"Validation Exception: {str(exc)}")
    return JSONResponse(
        status_code=422,
        content={"detail": "Validation error", "message": str(exc)}
    )


async def general_exception_handler(request: Request, exc: Exception):
    """Handle general exceptions"""
    logger.error(f"General Exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error"}
    )


def setup_error_handlers(app):
    """Setup error handlers for the FastAPI app"""
    app.add_exception_handler(HTTPException, http_exception_handler)
    app.add_exception_handler(Exception, general_exception_handler)