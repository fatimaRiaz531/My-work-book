# Quickstart Guide: Integrated RAG Chatbot

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Docker and Docker Compose (optional, for containerized deployment)
- Qdrant Cloud account with API key
- Neon Serverless Postgres account with connection string
- OpenAI API key

## Environment Setup

1. Create a `.env` file in the project root with the following variables:
```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
SECRET_KEY=your_secret_key_for_session_management
```

## Local Development Setup

### Backend Setup

1. Navigate to the backend directory: `cd backend`
2. Install Python dependencies: `pip install -r requirements.txt`
3. Set up the database: `python -m src.config.database init`
4. Start the backend: `uvicorn src.api.main:app --reload`

### Frontend Setup

1. Navigate to the frontend directory: `cd frontend/chat-widget`
2. Install Node dependencies: `npm install`
3. Build the widget: `npm run build`
4. The built widget will be available in the `dist/` directory

### Document Ingestion

1. Navigate to the ingestion directory: `cd ingestion`
2. Run the ingestion pipeline: `python -m src.main`
3. This will parse your Docusaurus content, chunk it, and store embeddings in Qdrant

## API Usage Examples

### Global Mode Query
```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Authorization: Bearer your_api_key" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What does the book say about RAG systems?",
    "session_id": "optional_session_id"
  }'
```

### Selection Mode Query
```bash
curl -X POST http://localhost:8000/api/v1/chat/selection \
  -H "Authorization: Bearer your_api_key" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this concept in simpler terms",
    "selection": "The Retrieval-Augmented Generation (RAG) system combines a retrieval component that fetches relevant documents and a generation component that creates responses based on the retrieved context.",
    "session_id": "optional_session_id"
  }'
```

### Health Check
```bash
curl -X GET http://localhost:8000/api/v1/health
```

## Embedding in Docusaurus

To embed the chatbot in your Docusaurus site:

1. Copy the `frontend/embed/chat-embed.js` file to your Docusaurus static assets
2. Add the script to your Docusaurus configuration
3. The widget will automatically detect selected text and enable the "Ask about selection" flow

## Testing

Run the test suite:
```bash
# Backend tests
cd backend
pytest

# Frontend tests
cd frontend/chat-widget
npm test
```

## Production Deployment

1. Build the frontend widget: `cd frontend/chat-widget && npm run build:prod`
2. Containerize the backend: `docker build -t rag-chatbot .`
3. Deploy to your preferred cloud platform with the environment variables set
4. Configure your domain and SSL certificates