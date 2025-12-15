# Implementation Plan: Integrated RAG Chatbot for Published Technical Book

**Branch**: `001-integrated-rag-chatbot` | **Date**: 2025-12-14 | **Spec**: specs/001-integrated-rag-chatbot/spec.md
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Production-ready Retrieval-Augmented Generation (RAG) chatbot with dual answering modes (full-book and selected-text-only) integrated with OpenAI Agents SDK, FastAPI backend, Neon Postgres for metadata, and Qdrant Cloud for vector search. The system will be embeddable into a Docusaurus book with responses grounded strictly in retrieved context.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant, SQLAlchemy, Pydantic, Requests
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (metadata/session storage)
**Testing**: pytest with integration and unit tests
**Target Platform**: Linux server (web deployment)
**Project Type**: web - backend API with embeddable frontend widget
**Performance Goals**: <5 second response time for queries, handle 100 concurrent requests
**Constraints**: <200ms p95 for API endpoints, responses must be grounded in book content only, secure API key management via environment variables
**Scale/Scope**: Single book corpus, 1000+ users, embeddable widget for Docusaurus sites

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Accuracy**: All responses must be grounded strictly in book content - IMPLEMENTED via RAG pipeline with strict context injection
- **Faithfulness**: No hallucination; answers must be traceable to retrieved chunks - IMPLEMENTED via context-only answering mechanism
- **Reproducibility**: RAG pipeline must be fully reproducible locally and in deployment - IMPLEMENTED via documented setup and environment configuration
- **Security**: API keys must be handled via environment variables only - IMPLEMENTED via secure configuration management
- **Accessibility**: Support full book and selected text answering modes - IMPLEMENTED via dual query modes (Global and Selection)

## Project Structure

### Documentation (this feature)

```text
specs/001-integrated-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── embedding.py
│   │   ├── document.py
│   │   ├── session.py
│   │   └── response.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── retrieval_service.py
│   │   ├── rag_service.py
│   │   └── document_service.py
│   ├── api/
│   │   ├── chat_router.py
│   │   ├── health_router.py
│   │   └── main.py
│   ├── config/
│   │   ├── settings.py
│   │   └── database.py
│   └── utils/
│       ├── text_chunker.py
│       └── citation_formatter.py
├── tests/
│   ├── unit/
│   │   ├── test_embedding_service.py
│   │   ├── test_retrieval_service.py
│   │   └── test_rag_service.py
│   ├── integration/
│   │   ├── test_chat_endpoints.py
│   │   └── test_health_endpoints.py
│   └── contract/
│       └── test_api_contracts.py
├── requirements.txt
├── Dockerfile
├── docker-compose.yml
└── README.md

frontend/
├── chat-widget/
│   ├── src/
│   │   ├── components/
│   │   │   ├── ChatInterface.jsx
│   │   │   ├── Message.jsx
│   │   │   └── SelectionHandler.jsx
│   │   ├── services/
│   │   │   └── api-client.js
│   │   └── styles/
│   │       └── chat-widget.css
│   ├── dist/
│   └── package.json
└── embed/
    └── chat-embed.js

ingestion/
├── src/
│   ├── processors/
│   │   ├── docusaurus_parser.py
│   │   ├── text_chunker.py
│   │   └── embedding_generator.py
│   ├── main.py
│   └── config.py
└── scripts/
    └── run_ingestion.py
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (embeddable widget) components. Backend handles API, RAG logic, and data processing. Frontend provides embeddable chat widget for Docusaurus integration. Ingestion pipeline handles document processing and vector storage.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Dual storage systems (Qdrant + Postgres) | Vector DB for semantic search, relational DB for metadata/session tracking | Single DB insufficient for semantic retrieval requirements |
| Multiple components (backend, frontend, ingestion) | Clear separation of concerns and specialized functionality | Monolithic approach would compromise maintainability and scalability |
