# Integrated RAG Chatbot for AI-Native Technical Book Constitution

## Core Principles

### I. Groundedness and Zero Hallucination
All responses must be derived strictly from retrieved content. If information is not present in the provided content, explicitly state "The requested information is not present in the provided content." No exceptions.

### II. Context Isolation
Selected-text queries must use only user-provided text as context, ignoring all global embeddings. Global queries must use only the book's content, with strict separation between modes.

### III. Determinism & Reproducibility
Same inputs must yield consistent outputs. The system must be reproducible with one-command local startup and all setup steps documented. No proprietary or opaque dependencies allowed.

### IV. LLM & AI Constraints
Use Cohere language and embedding models exclusively (no OpenAI APIs). Prompting must be explicit, minimal, and auditable. No hidden system prompts or undocumented logic.

### V. Production-Ready Quality
Code must be production-ready with clear module boundaries, fully documented API endpoints, and clean separation of retrieval, generation, and UI layers.

### VI. Transparency and Attribution
Every response must clearly reference source sections or chapters when applicable. Responses must be concise, technically accurate, and well-structured.

## Technical Constraints

### Technology Stack
- Backend Framework: FastAPI
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless Postgres
- Frontend: Docusaurus with embedded chatbot UI
- RAG Pipeline: Chunking with deterministic rules, metadata-rich embeddings (chapter, section, page), Top-k retrieval with score thresholding

### Security and Data Handling
- Never hardcode secrets or tokens; use .env and docs
- All data handling must follow security best practices
- User privacy must be protected in all interactions

## Development Workflow

### Code Standards
- Prefer the smallest viable diff; do not refactor unrelated code
- Cite existing code with code references
- All changes must be testable and include appropriate error handling
- Follow established patterns in the codebase

### Testing Requirements
- All features must have appropriate unit and integration tests
- RAG functionality must be validated for accuracy and grounding
- Selected-text mode must be thoroughly tested for context isolation

## Governance

This constitution supersedes all other practices. All PRs/reviews must verify compliance with these principles. Complexity must be justified, and architectural decisions must be documented with ADRs when significant.

**Version**: 1.0.0 | **Ratified**: 2025-12-18 | **Last Amended**: 2025-12-18
