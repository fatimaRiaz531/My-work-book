---
description: "Task list for Integrated RAG Chatbot implementation"
---

# Tasks: Integrated RAG Chatbot for Published Technical Book

**Input**: Design documents from `/specs/001-integrated-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in backend/, frontend/, ingestion/
- [x] T002 Initialize Python project with FastAPI, OpenAI Agents SDK, Qdrant, SQLAlchemy dependencies in backend/requirements.txt
- [ ] T003 [P] Configure linting and formatting tools (black, flake8, mypy) in backend/
- [ ] T004 [P] Initialize Node.js project with React dependencies in frontend/chat-widget/package.json
- [x] T005 Create Dockerfile and docker-compose.yml for containerized deployment
- [x] T006 Set up environment configuration management in backend/src/config/settings.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Setup database schema and migrations framework for Neon Postgres in backend/src/config/database.py
- [x] T008 [P] Implement Qdrant vector database connection and collection setup in backend/src/config/database.py
- [x] T009 [P] Setup API routing and middleware structure in backend/src/api/main.py
- [x] T010 Create base models/entities that all stories depend on in backend/src/models/
- [x] T011 Configure error handling and logging infrastructure in backend/src/config/
- [x] T012 [P] Create configuration for OpenAI API integration in backend/src/config/settings.py
- [x] T013 Set up basic ingestion pipeline structure in ingestion/src/

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Global Book Search (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about the entire book content and receive comprehensive answers from the full knowledge base with source citations.

**Independent Test**: Can be fully tested by asking questions about general book topics and verifying that responses are accurate, grounded in the book content, and include proper citations.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T014 [P] [US1] Contract test for /chat endpoint in backend/tests/contract/test_chat_api.py
- [ ] T015 [P] [US1] Integration test for global book search journey in backend/tests/integration/test_global_search.py

### Implementation for User Story 1

- [x] T016 [P] [US1] Create Document model in backend/src/models/document.py
- [x] T017 [P] [US1] Create Session model in backend/src/models/session.py
- [x] T018 [P] [US1] Create Response model in backend/src/models/response.py
- [x] T019 [US1] Implement DocumentService in backend/src/services/document_service.py
- [x] T020 [US1] Implement RetrievalService in backend/src/services/retrieval_service.py
- [x] T021 [US1] Implement RAGService in backend/src/services/rag_service.py
- [x] T022 [US1] Implement /chat endpoint in backend/src/api/chat_router.py
- [x] T023 [US1] Add validation and error handling for global search
- [x] T024 [US1] Add citation formatting utilities in backend/src/utils/citation_formatter.py
- [x] T025 [US1] Add logging for global search operations

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Contextual Text Selection Search (Priority: P2)

**Goal**: Enable users to select specific text on a page and ask questions about only that selected text to get focused answers from the selected context.

**Independent Test**: Can be fully tested by selecting text on a page, activating the chat interface, and verifying that answers are based only on the selected text with no reference to other book content.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T026 [P] [US2] Contract test for /chat/selection endpoint in backend/tests/contract/test_selection_api.py
- [ ] T027 [P] [US2] Integration test for text selection search journey in backend/tests/integration/test_selection_search.py

### Implementation for User Story 2

- [x] T028 [P] [US2] Create TextSelection model in backend/src/models/text_selection.py
- [x] T029 [US2] Extend RAGService to support selection-only mode in backend/src/services/rag_service.py
- [x] T030 [US2] Implement /chat/selection endpoint in backend/src/api/chat_router.py
- [x] T031 [US2] Add validation to ensure selection mode ignores other documents
- [ ] T032 [US2] Implement text selection handling in frontend/chat-widget/src/services/api-client.js
- [ ] T033 [US2] Create SelectionHandler component in frontend/chat-widget/src/components/SelectionHandler.jsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - API Access to RAG System (Priority: P3)

**Goal**: Provide API endpoints for developers to access the RAG system and integrate chatbot functionality into different interfaces.

**Independent Test**: Can be fully tested by making direct API calls to the endpoints and verifying that responses are properly formatted and contain relevant information.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T034 [P] [US3] Contract test for /health endpoint in backend/tests/contract/test_health_api.py
- [ ] T035 [P] [US3] Integration test for API access patterns in backend/tests/integration/test_api_access.py

### Implementation for User Story 3

- [x] T036 [P] [US3] Create Embedding model in backend/src/models/embedding.py
- [x] T037 [US3] Implement EmbeddingService in backend/src/services/embedding_service.py
- [x] T038 [US3] Implement /health endpoint in backend/src/api/health_router.py
- [ ] T039 [US3] Add comprehensive API documentation and validation
- [ ] T040 [US3] Implement session tracking and management in backend/src/services/session_service.py
- [ ] T041 [US3] Add rate limiting middleware in backend/src/api/main.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Ingestion Pipeline Implementation

**Goal**: Implement the document processing pipeline to parse book content, chunk text, generate embeddings, and store in Qdrant and Postgres.

- [x] T042 [P] Create Docusaurus parser in ingestion/src/processors/docusaurus_parser.py
- [x] T043 [P] Implement text chunker with overlap in ingestion/src/processors/text_chunker.py
- [x] T044 Create embedding generator using OpenAI API in ingestion/src/processors/embedding_generator.py
- [x] T045 Implement document ingestion pipeline in ingestion/src/main.py
- [x] T046 Add ingestion configuration and settings in ingestion/src/config.py
- [x] T047 Create ingestion runner script in ingestion/scripts/run_ingestion.py
- [x] T048 Add ingestion validation and error handling

---

## Phase 7: Frontend Integration

**Goal**: Create embeddable chatbot UI that works seamlessly within the book site and supports text selection flow.

- [x] T049 [P] Create ChatInterface component in frontend/chat-widget/src/components/ChatInterface.jsx
- [x] T050 [P] Create Message component in frontend/chat-widget/src/components/Message.jsx
- [x] T051 Create API client service in frontend/chat-widget/src/services/api-client.js
- [x] T052 Add chat widget styling in frontend/chat-widget/src/styles/chat-widget.css
- [x] T053 Create embed script for Docusaurus integration in frontend/embed/chat-embed.js
- [x] T054 Implement text selection → "Ask about selection" flow in frontend/chat-widget/src/components/SelectionHandler.jsx

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T055 [P] Documentation updates in README.md and docs/
- [ ] T056 Code cleanup and refactoring across all components
- [ ] T057 Performance optimization for API endpoints and queries
- [ ] T058 [P] Additional unit tests in backend/tests/unit/ and frontend/chat-widget/tests/
- [x] T059 Security hardening and input validation
- [x] T060 Run quickstart.md validation and update as needed
- [ ] T061 Add comprehensive error handling and user feedback
- [ ] T062 Optimize vector search performance and relevance

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Ingestion Pipeline (Phase 6)**: Can run in parallel with user stories after foundational phase
- **Frontend Integration (Phase 7)**: Can start after foundational phase but may depend on API completion
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for /chat endpoint in backend/tests/contract/test_chat_api.py"
Task: "Integration test for global book search journey in backend/tests/integration/test_global_search.py"

# Launch all models for User Story 1 together:
Task: "Create Document model in backend/src/models/document.py"
Task: "Create Session model in backend/src/models/session.py"
Task: "Create Response model in backend/src/models/response.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence