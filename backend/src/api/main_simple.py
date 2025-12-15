from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
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

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Health check endpoint
@app.get("/api/v1/health")
async def health_check():
    return {"status": "healthy", "service": "RAG Chatbot API"}

# Health check endpoint with details
@app.get("/api/v1/health/details")
async def health_check_detailed():
    return {
        "status": "healthy",
        "service": "RAG Chatbot API",
        "dependencies": {
            "qdrant": "not connected (requires API key)",
            "database": "not connected (requires configuration)",
            "openai": "not connected (requires API key)"
        }
    }

@app.on_event("startup")
async def startup_event():
    logging.info("Application startup complete")

@app.on_event("shutdown")
async def shutdown_event():
    logging.info("Application shutdown complete")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)