# Integrated RAG Chatbot for Published Technical Book

This project implements a production-ready Retrieval-Augmented Generation (RAG) chatbot that allows users to ask questions about a technical book and receive accurate, cited answers based on the book's content.

## Features

- **Dual Query Modes**: Full-book search and selection-based queries
- **Context-Aware Responses**: Answers based only on provided context with strict adherence to source material
- **Source Citations**: All answers include references to specific sections, pages, and document sources
- **Embeddable Widget**: Easy integration with Docusaurus and other documentation sites
- **Text Selection Capture**: Automatically detects and allows querying of selected text
- **Secure API**: Rate limiting, input validation, and authentication
- **Production Ready**: Multiple deployment options with health monitoring
- **Session Management**: Persistent conversation tracking
- **Fallback Handling**: Graceful responses for unsupported queries

## Architecture

The system consists of four main components:

1. **Backend**: FastAPI application handling RAG logic and API endpoints
2. **Ingestion Pipeline**: Processes book content and generates embeddings
3. **Frontend Widget**: Embeddable chat interface with text selection support
4. **Deployment Infrastructure**: Containerized and serverless deployment options

## Tech Stack

- **Backend**: Python 3.11, FastAPI, SQLAlchemy, Pydantic
- **LLM Integration**: OpenAI API (GPT-3.5-turbo, text-embedding-ada-002)
- **Vector Database**: Qdrant Cloud
- **Relational Database**: Neon Serverless Postgres
- **Frontend**: React 18, JavaScript, CSS
- **Deployment**: Docker, docker-compose, Serverless Framework, Nginx

## Prerequisites

- Python 3.11+
- Node.js 18+ (for frontend development)
- Docker and Docker Compose (for containerized deployment)
- Qdrant Cloud account with API key
- Neon Serverless Postgres account with connection string
- OpenAI API key

## Setup

### Environment Configuration

Create a `.env` file in the project root with the following variables:

```bash
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
SECRET_KEY=your_secret_key_for_session_management
POSTGRES_PASSWORD=your_postgres_password
```

### Backend Setup

1. Navigate to the backend directory: `cd backend`
2. Install Python dependencies: `pip install -r requirements.txt`
3. Start the backend: `uvicorn src.api.main:app --reload`

### Frontend Setup

1. Navigate to the frontend directory: `cd frontend/chat-widget`
2. Install Node dependencies: `npm install`
3. Build the widget: `npm run build`

### Document Ingestion

1. Navigate to the ingestion directory: `cd ingestion`
2. Run the ingestion pipeline: `python -m src.main`

## API Endpoints

### Health Check
- `GET /api/v1/health` - Check service health
- `GET /api/v1/health?include_details=true` - Detailed connection validation

### Chat Endpoints
- `POST /api/v1/chat` - Global book search
- `POST /api/v1/chat/selection` - Selection-based search with user-provided text

## Frontend Integration

To embed the chatbot in your Docusaurus site:

1. Copy the `frontend/embed/chat-embed.js` file to your Docusaurus static assets
2. Add the script to your Docusaurus configuration
3. The widget will automatically detect selected text and enable the "Ask about selection" flow

### Configuration Options
- Set API URL: `window.RAG_CHATBOT_API_URL = 'your-api-url'`
- Set widget URL: `window.RAG_CHATBOT_WIDGET_URL = 'your-widget-url'`

## Development

### Running with Docker (Recommended)

Use the provided `docker-compose.yml` to run the entire stack:

```bash
docker-compose up --build
```

For production deployment:
```bash
docker-compose -f docker-compose.prod.yml up -d
```

### Running with Serverless

Deploy to AWS Lambda:
```bash
# Install serverless framework
npm install -g serverless

# Deploy to AWS
serverless deploy
```

### Testing

Run the backend tests:
```bash
cd backend
pytest
```

### Frontend Development

```bash
cd frontend/chat-widget
npm run dev  # Development server
npm run build  # Production build
npm run build:dev  # Development build
```

## Deployment Options

### 1. Containerized Deployment (Docker Compose)

The default `docker-compose.yml` includes all necessary services:

```bash
docker-compose up -d
```

### 2. Production Containerized Deployment

Use the production configuration:

```bash
docker-compose -f docker-compose.prod.yml up -d
```

### 3. Serverless Deployment (AWS Lambda)

Deploy using Serverless Framework:

```bash
serverless deploy
```

### 4. Traditional Deployment

Build the frontend and deploy the backend as a standard FastAPI application:

```bash
# Build frontend
cd frontend/chat-widget && npm run build

# Deploy backend with nginx reverse proxy
# Use the provided nginx.conf for production setup
```

## Health Monitoring

The system includes comprehensive health monitoring:

- Basic health check: `GET /api/v1/health`
- Detailed validation: `GET /api/v1/health?include_details=true`
- Validates connections to Qdrant, PostgreSQL, and OpenAI
- Returns status of all dependencies

## Security Features

- Input validation and sanitization
- Rate limiting to prevent API abuse
- Secure API key management
- CORS configuration for web integration
- Proper error handling without information disclosure

## File Structure

```
backend/                 # FastAPI backend
├── src/
│   ├── api/            # API routes and main application
│   ├── services/       # Core services (RAG, retrieval, document)
│   ├── models/         # Data models and schemas
│   ├── config/         # Configuration and database setup
│   └── utils/          # Utility functions
├── requirements.txt    # Python dependencies
├── serverless_handler.py # Serverless compatibility
└── test_basic.py       # Basic tests

frontend/               # Frontend components
├── chat-widget/        # React chat interface
│   ├── src/
│   │   ├── components/ # React components
│   │   ├── services/   # API services
│   │   └── styles/     # CSS styles
│   ├── webpack.config.js # Build configuration
│   └── package.json    # Frontend dependencies
└── embed/              # Embeddable script
    └── chat-embed.js   # Docusaurus integration script

ingestion/              # Data ingestion pipeline
├── src/
│   ├── processors/     # Document parsing and processing
│   └── main.py         # Pipeline entry point
└── scripts/            # Helper scripts

Dockerfile             # Backend container
docker-compose.yml     # Local development orchestration
docker-compose.prod.yml # Production orchestration
serverless.yml         # Serverless deployment config
nginx.conf             # Production web server config
```

## Troubleshooting

### Common Issues

1. **Connection Issues**: Verify all environment variables are set correctly
2. **Embedding Issues**: Check OpenAI API key and rate limits
3. **Database Issues**: Ensure PostgreSQL and Qdrant are accessible
4. **Frontend Build Issues**: Verify Node.js version and dependencies

### Health Check

Run the health check endpoint to verify all services:

```bash
curl http://localhost:8000/api/v1/health?include_details=true
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Update documentation
6. Submit a pull request

## License

MIT

