<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles: All principles updated for RAG Chatbot project
Added sections: Technical standards section with backend, LLM integration, databases, and frontend requirements
Removed sections: Old book creation specific sections
Templates requiring updates: ✅ Updated /specify/templates/plan-template.md, ✅ Updated /specify/templates/spec-template.md, ✅ Updated /specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Integrated RAG Chatbot for a Published Technical Book — Constitution

## Core Principles

### I. Accuracy

All chatbot answers must be grounded strictly in the book content. No information outside the book should be provided by the chatbot.

### II. Faithfulness

No hallucination; answers must be traceable to retrieved chunks. Every response must be verifiable against the source material.

### III. Reproducibility

RAG pipeline must be fully reproducible locally and in deployment. All processes should be documented and repeatable.

### IV. Security

API keys must be handled via environment variables only. No hardcoded credentials or secrets in the codebase.

### V. Accessibility

Chatbot must support answering from full book and user-selected text. Multiple interaction modes should be available.

## Technical Standards

- **Backend:** FastAPI (Python)
- **LLM Integration:** OpenAI Agents SDK / ChatKit SDK
- **Vector Database:** Qdrant Cloud (Free Tier)
- **Relational Database:** Neon Serverless Postgres
- **Frontend Embed:** Docusaurus-compatible widget or iframe
- **Retrieval:** Semantic vector search + optional metadata filtering
- **Citation:** Responses must reference source sections/pages when applicable

## Technical Requirements

- **Build Structure:** Use Spec-Kit Plus with `/spec/features`, `/spec/system`, `/spec/api`, `/spec/models`.
- **Content Processing:** Implement document chunking and embedding pipeline.
- **Publishing:** Integrate chatbot widget into Docusaurus documentation pages.
- **Deployment:** Deploy to cloud platform with secure API key management.
- **Final Format:** Embeddable chat widget with full RAG functionality.

## Content Quality Requirements

- **Grounding:** All responses must be grounded in book content.
- **Traceability:** Citations must point to specific sections/pages in the source book.
- **Relevance:** Responses must be contextually relevant to user queries.
- **Completeness:** Provide comprehensive answers based on available content.
- **Consistency:** Maintain consistent tone and accuracy across all responses.

## Success Criteria

- Chatbot fully integrated with RAG pipeline.
- All responses verified with source citations and grounding requirements.
- Build passes deployment and security checks.
- Widget integrates seamlessly with Docusaurus documentation.
- Meets all accuracy, faithfulness, and security requirements.

## Governance

This constitution supersedes all other project practices. Amendments require documentation, approval, and a migration plan. All PRs/reviews must verify compliance. Complexity must be justified.

**Version**: 1.1.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
