# Data Model: Integrated RAG Chatbot for Published Technical Book

## Document Entity

**Document** represents a book content chunk with metadata for retrieval

- `id`: UUID - Unique identifier for the document chunk
- `content`: Text - The actual content of the document chunk
- `source_file`: Text - Original file path from Docusaurus project
- `section`: Text - Section name or heading
- `chapter`: Text - Chapter name if applicable
- `page_number`: Integer - Page number if available
- `embedding_vector`: Vector - Embedding vector for semantic search
- `created_at`: DateTime - When the document was indexed
- `updated_at`: DateTime - When the document was last updated

**Relationships**: None direct, but linked via semantic similarity

## Session Entity

**Session** represents a user's chat session for tracking conversation context

- `id`: UUID - Unique identifier for the session
- `session_token`: Text - Secure session identifier
- `user_id`: Text - Optional user identifier (anonymous if null)
- `created_at`: DateTime - When the session started
- `updated_at`: DateTime - When the session was last active
- `expires_at`: DateTime - When the session expires

**Relationships**: One-to-many with Message entities

## Message Entity

**Message** represents a single message in a chat conversation

- `id`: UUID - Unique identifier for the message
- `session_id`: UUID - Reference to the parent session
- `role`: Text - Either "user" or "assistant"
- `content`: Text - The message content
- `sources`: JSON - List of source documents referenced in the response
- `created_at`: DateTime - When the message was created
- `sequence_number`: Integer - Order of the message in the conversation

**Relationships**: Belongs to one Session entity

## Embedding Entity

**Embedding** represents vector embeddings for semantic search (stored in Qdrant)

- `id`: UUID - Unique identifier for the embedding
- `document_id`: UUID - Reference to the source document
- `vector`: Vector - The embedding vector (handled by Qdrant)
- `metadata`: JSON - Additional metadata for filtering (source_file, section, etc.)
- `created_at`: DateTime - When the embedding was generated

**Relationships**: Links to Document entity

## User Entity

**User** represents optional user accounts for enhanced features

- `id`: UUID - Unique identifier for the user
- `email`: Text - User's email address (optional for anonymous use)
- `created_at`: DateTime - When the user account was created
- `preferences`: JSON - User preferences for the chat experience

**Relationships**: One-to-many with Session entities