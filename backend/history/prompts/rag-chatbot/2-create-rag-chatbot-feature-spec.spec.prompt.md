---
id: 2
title: Create RAG Chatbot Feature Spec
stage: spec
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot
branch: 1-rag-chatbot
user: user
command: /sp.specify
labels: [spec, rag, chatbot, docusaurus]
links:
  spec: specs/1-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-rag-chatbot/spec.md
 - specs/1-rag-chatbot/checklists/requirements.md
tests:
 -
---

## Prompt

Integrated RAG Chatbot for Docusaurus Technical Book

Target audience:
- Software engineers
- AI/ML practitioners
- Technical readers of the published book
- Students using the book as a learning reference

Primary objective:
Design and implement a Retrieval-Augmented Generation (RAG) chatbot embedded inside a Docusaurus-based book that answers questions strictly from the book's content, using Cohere LLMs and embeddings. The system must support a "selected-text only" answering mode with zero hallucination.

Technology stack (fixed):
- LLM Provider: Cohere (text generation + embeddings)
- Backend API: FastAPI
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless Postgres
- Frontend: Docusaurus (embedded chatbot UI)
- AI Orchestration: Spec-Kit Plus + Claude CLI

Environment configuration (required):
- QDRANT_API_KEY (from environment variables)
- QDRANT_URL (Qdrant Cloud HTTPS endpoint)
- COHERE_API_KEY (from environment variables)
- NEON_DATABASE_URL (Postgres connection string)

RAG capabilities:
- Index all book chapters using deterministic chunking
- Store embeddings with metadata:
  - module
  - chapter
  - section
  - heading hierarchy
- Perform top-k semantic retrieval with similarity thresholds
- Generate answers strictly from retrieved content

Selected-text answering mode:
- Accept user-highlighted text from the UI
- Bypass vector database retrieval entirely
- Use only the provided text as generation context
- Explicitly prohibit use of global book embeddings
- If answer is not present in selected text, respond:
  "The answer is not contained in the selected content."

Answering rules:
- No external knowledge
- No assumptions beyond retrieved text
- No hallucinations
- If retrieval confidence is insufficient, explicitly say so
- Answers must be concise, technical, and source-grounded

Backend API requirements:
- /ingest endpoint for book content indexing
- /query endpoint for global RAG queries
- /query/selection endpoint for selected-text queries
- Clear request/response schemas
- Deterministic behavior across runs

Data & storage rules:
- Qdrant used only for embeddings + metadata
- Neon Postgres used for:
  - chat logs
  - query metadata
  - feedback signals (optional)
- No vector data stored in Postgres

Documentation & reproducibility:
- All setup steps documented
- .env.example file provided
- One-command local startup
- No hidden configuration or prompts

Success criteria:
- Chatbot answers are always grounded in book content
- Selected-

## Response snapshot

Successfully created the feature specification for the Integrated RAG Chatbot. The specification includes:

1. **User Scenarios & Testing**:
   - Primary user story for asking questions about book content (P1)
   - Selected-text only mode functionality (P2)
   - Content indexing capabilities (P2)
   - Chat history and logging (P3)

2. **Functional Requirements**:
   - 13 detailed functional requirements covering all core functionality
   - Requirements for content retrieval, answer generation, and context isolation
   - API endpoints for ingestion and querying

3. **Success Criteria**:
   - 6 measurable outcomes with specific metrics
   - Focus on accuracy, response time, and context isolation

4. **Quality Assurance**:
   - Created a comprehensive quality checklist
   - Removed all implementation-specific details (Cohere, Qdrant, Postgres, etc.)
   - Made the specification technology-agnostic while preserving user value

The specification is now ready for the planning phase and meets all quality criteria.

## Outcome

- ✅ Impact: Complete feature specification created with user scenarios, requirements, and success criteria
- 🧪 Tests: N/A
- 📁 Files: Created specs/1-rag-chatbot/spec.md and specs/1-rag-chatbot/checklists/requirements.md
- 🔁 Next prompts: Ready for /sp.plan to create implementation plan
- 🧠 Reflection: Specification successfully abstracted technology details while preserving core functionality

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All checklist items validated as complete
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin architectural planning with /sp.plan