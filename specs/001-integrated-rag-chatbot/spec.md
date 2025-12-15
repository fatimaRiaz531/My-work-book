# Feature Specification: Integrated RAG Chatbot for Published Technical Book

**Feature Branch**: `001-integrated-rag-chatbot`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: "System behavior specification: 1. Ingestion Pipeline: - Parse all book content (Markdown/MDX) from the Docusaurus project - Chunk text with overlap for semantic retrieval - Generate embeddings using OpenAI embeddings - Store embeddings and metadata (chapter, section, file path) in Qdrant - Store document index and session logs in Neon Postgres 2. Query Modes: - Global Mode: Answer questions using the entire book corpus - Selection Mode: Answer questions using ONLY user-selected text - Selection Mode must ignore all other documents 3. RAG Logic: - Retrieve top-k relevant chunks from Qdrant - Inject retrieved context into OpenAI Agent prompt - Enforce context-only answering - If answer is not found, respond with "Not found in provided content" 4. API Layer: - FastAPI endpoints for: - /chat - /chat/selection - /health - Stateless request handling with optional session tracking 5. Frontend Integration: - Lightweight chatbot UI embeddable inside the book site - Support text selection → "Ask about selection" flow"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Global Book Search (Priority: P1)

As a reader of the technical book, I want to ask questions about the entire book content so that I can get comprehensive answers from the full knowledge base.

**Why this priority**: This provides the core value of the RAG system - enabling users to ask questions about the entire book and get accurate answers based on the book's content.

**Independent Test**: Can be fully tested by asking questions about general book topics and verifying that responses are accurate, grounded in the book content, and include proper citations.

**Acceptance Scenarios**:

1. **Given** I am on a book page with the chatbot interface, **When** I ask a question about the book content, **Then** I receive an answer based on the entire book corpus with source citations.
2. **Given** I ask a question not found in the book content, **When** I submit the query, **Then** I receive the response "Not found in provided content".

---

### User Story 2 - Contextual Text Selection Search (Priority: P2)

As a reader, I want to select specific text on a page and ask questions about only that selected text so that I can get focused answers from the selected context.

**Why this priority**: This provides a more focused search capability that allows users to get answers based on specific sections they're currently reading.

**Independent Test**: Can be fully tested by selecting text on a page, activating the chat interface, and verifying that answers are based only on the selected text with no reference to other book content.

**Acceptance Scenarios**:

1. **Given** I have selected text on a book page, **When** I ask a question using the "Ask about selection" flow, **Then** I receive an answer based only on the selected text.
2. **Given** I have selected text on a book page, **When** I ask a question that cannot be answered from the selected text, **Then** I receive the response "Not found in provided content".

---

### User Story 3 - API Access to RAG System (Priority: P3)

As a developer, I want to access the RAG system through API endpoints so that I can integrate the chatbot functionality into different interfaces.

**Why this priority**: This enables the technical infrastructure needed to support both global and selection-based queries.

**Independent Test**: Can be fully tested by making direct API calls to the endpoints and verifying that responses are properly formatted and contain relevant information.

**Acceptance Scenarios**:

1. **Given** I make a POST request to /chat endpoint, **When** I provide a query about the book content, **Then** I receive a properly formatted response with citations from the book.
2. **Given** I make a POST request to /chat/selection endpoint, **When** I provide selected text and a query, **Then** I receive a response based only on the provided text.

---

### Edge Cases

- What happens when the selected text is empty or invalid?
- How does the system handle very long text selections that exceed embedding limits?
- What happens when the book content is updated and embeddings need to be refreshed?
- How does the system handle concurrent users making multiple requests simultaneously?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST parse all book content (Markdown/MDX) from the Docusaurus project during ingestion
- **FR-002**: System MUST chunk text with overlap for semantic retrieval during ingestion
- **FR-003**: System MUST generate embeddings using OpenAI embeddings API during ingestion
- **FR-004**: System MUST store embeddings and metadata (chapter, section, file path) in Qdrant vector database
- **FR-005**: System MUST store document index and session logs in Neon Postgres database
- **FR-006**: System MUST support Global Mode that answers questions using the entire book corpus
- **FR-007**: System MUST support Selection Mode that answers questions using ONLY user-selected text
- **FR-008**: System MUST ensure Selection Mode ignores all other documents not in the selected text
- **FR-009**: System MUST retrieve top-k relevant chunks from Qdrant during query processing
- **FR-010**: System MUST inject retrieved context into OpenAI Agent prompt for answer generation
- **FR-011**: System MUST enforce context-only answering based on provided context
- **FR-012**: System MUST respond with "Not found in provided content" when answer is not found in the provided context
- **FR-013**: System MUST provide FastAPI endpoint at /chat for global queries
- **FR-014**: System MUST provide FastAPI endpoint at /chat/selection for selection-based queries
- **FR-015**: System MUST provide FastAPI endpoint at /health for system health checks
- **FR-016**: System MUST handle requests statelessly with optional session tracking
- **FR-017**: System MUST provide a lightweight chatbot UI embeddable inside the book site
- **FR-018**: System MUST support text selection → "Ask about selection" flow in the frontend

### Key Entities

- **Book Content**: The source material from the Docusaurus project, consisting of Markdown/MDX files with chapters, sections, and file paths
- **Embeddings**: Vector representations of text chunks stored in Qdrant for semantic retrieval
- **Query Session**: Optional tracking information for user interactions, stored in Neon Postgres
- **Chat Response**: Generated answers with proper citations to source sections/pages in the book
- **Text Selection**: User-specified text range that limits the context for Selection Mode queries

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can ask questions about the book content and receive accurate, cited answers within 5 seconds
- **SC-002**: The system successfully processes 95% of queries with relevant answers from the book content
- **SC-003**: Users can select text on a page and ask questions about only that text with 95% accuracy in response relevance
- **SC-004**: The API endpoints handle 100 concurrent requests without performance degradation
- **SC-005**: The chatbot UI integrates seamlessly into the book site without affecting page load times by more than 200ms
- **SC-006**: 90% of users successfully complete both global and selection-based queries on first attempt
