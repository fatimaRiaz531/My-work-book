from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from .chat_router import router as chat_router
from .health_router import router as health_router
from ..config.error_handlers import setup_error_handlers
from .middleware import rate_limit
import logging
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)

app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot for technical book queries",
    version="1.0.0"
)

# Setup error handlers
setup_error_handlers(app)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiting middleware
@app.middleware("http")
async def add_rate_limiting(request: Request, call_next):
    # Apply rate limiting to all endpoints except health check
    if request.url.path != "/api/v1/health":
        rate_limit(max_requests=100, window_seconds=3600)(request)
    response = await call_next(request)
    return response

# Include routers
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(health_router, prefix="/api/v1", tags=["health"])

# Add a root endpoint for easy testing
@app.get("/")
async def root():
    return {
        "message": "RAG Chatbot API is running",
        "endpoints": [
            "/api/v1/chat - General chat endpoint",
            "/api/v1/chat/selection - Selection-based chat",
            "/api/v1/health - Health check",
            "/api/v1/ready - Readiness check",
            "/docs - API documentation"
        ]
    }

@app.on_event("startup")
async def startup_event():
    # Ensure Qdrant has sample data on startup
    from ..config.database import get_qdrant
    qdrant_client = get_qdrant()
    logging.info("Application startup complete")

@app.on_event("shutdown")
async def shutdown_event():
    # Any cleanup logic can go here
    logging.info("Application shutdown complete")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)