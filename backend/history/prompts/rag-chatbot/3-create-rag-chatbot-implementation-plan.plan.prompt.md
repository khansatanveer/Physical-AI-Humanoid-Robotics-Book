---
id: 3
title: Create RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot
branch: 1-rag-chatbot
user: user
command: /sp.plan
labels: [plan, architecture, rag, chatbot]
links:
  spec: specs/1-rag-chatbot/spec.md
  plan: specs/1-rag-chatbot/plan/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/1-rag-chatbot/plan/plan.md
 - specs/1-rag-chatbot/plan/research.md
 - specs/1-rag-chatbot/plan/data-model.md
 - specs/1-rag-chatbot/plan/contracts/openapi.yaml
 - specs/1-rag-chatbot/plan/quickstart.md
tests:
 -
---

## Prompt

Integrated RAG Chatbot for Docusaurus Technical Book

Objective:
Produce a step-by-step, implementation-ready plan to build a Cohere-based Retrieval-Augmented Generation (RAG) chatbot embedded within a Docusaurus-published technical book, fully aligned with the project constitution and sp.specify constraints.

Planning principles:
- Follow constitution rules strictly (groundedness, zero hallucination, reproducibility)
- Respect all technology constraints (Cohere only, no OpenAI)
- Plan must be deterministic, modular, and auditable
- No implementation code yet — planning and sequencing only

Phase 1: Repository & Environment Setup
- Define repository structure for:
  - backend (FastAPI)
  - ingestion pipeline
  - RAG logic
  - frontend (Docusaurus integration)
- Create .env.example with required variables:
  - COHERE_API_KEY
  - QDRANT_API_KEY
  - QDRANT_URL
  - NEON_DATABASE_URL
- Document local and production setup steps
- Ensure project is reproducible from repository alone

Phase 2: Book Content Ingestion & Chunking
- Extract Markdown/MDX content from Docusaurus book
- Design deterministic chunking strategy:
  - fixed token/character limits
  - overlap rules
- Attach metadata to each chunk:
  - module
  - chapter
  - section
  - heading path
- Validate chunk integrity and ordering

Phase 3: Embedding & Vector Storage (Qdrant)
- Generate embeddings using Cohere embedding models
- Store embeddings in Qdrant Cloud with metadata
- Define collection schema and similarity metric
- Implement idempotent ingestion (safe re-runs)

Phase 4: Global RAG Query Pipeline
- Accept user query
- Embed query using Cohere
- Perform top-k retrieval from Qdrant
- Apply similarity threshold filtering
- Assemble context strictly from retrieved chunks
- Generate answer using Cohere LLM
- Enforce grounded response rules

Phase 5: Selected-Text–Only Query Pipeline
- Accept user-selected text from Docusaurus UI
- Disable vector retrieval entirely
- Use selected text as sole context window
- Generate answer with explicit constraint:
  - no external or global knowledge allowed
- Return refusal if answer not present in selection

Phase 6: Backend API Design (FastAPI)
- Define endpoints:
  - POST /ingest
  - POST /query
  - POST /query/selection
- Specify request/response schemas
- Implement validation and error handling
- Ensure deterministic outputs for identical inputs

Phase 7: Neon Postgres Integration
- Design schema for:
  - chat logs
  - query metadata
  - timestamps
- Exclude embeddings from Postgres
- Ensure minimal, privacy-safe storage

Phase 8: Docusaurus Chatbot UI Integration
- Embed chatbot component into book layout
- Support:
  - normal query mode
  - selected-text query mode
- Clearly communicate source grounding to users
- Handle loading, errors, and empty responses

Phase 9: Prompt Design & Governance
- Define system and user prompt templates
- Explicitly encode:
  - zero hallucination rule
  - grounding requirement
- Ensure prompts are documented and versioned

Phase 10: Testing & Validation
- Test:
  - global RAG correctness
  - selected-text isolation
  - failure cases (missing info)
- Verify no external knowledge leakage
- Confi

## Response snapshot

Successfully created a comprehensive implementation plan for the Integrated RAG Chatbot. The plan includes:

1. **Technical Context**: Identified and resolved all unknowns through research
2. **Constitution Check**: Verified alignment with all project constitution principles
3. **Research Document**: Resolved all unknowns with specific technology decisions:
   - Cohere embed-multilingual-v3.0 for embeddings
   - Cohere command-r-plus for generation
   - 512-token chunking strategy
   - React component integration with Docusaurus
4. **Data Model**: Detailed entity definitions for BookContent, ChatInteraction, QueryLog, and UserFeedback
5. **API Contracts**: Complete OpenAPI specification for all endpoints
6. **Quickstart Guide**: Step-by-step setup and usage instructions

The implementation plan is organized into clear phases with specific tasks, risks, and success criteria. All constitution principles (groundedness, zero hallucination, context isolation) are addressed in the design.

## Outcome

- ✅ Impact: Complete implementation plan with resolved research questions and detailed architecture
- 🧪 Tests: N/A
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md
- 🔁 Next prompts: Ready for /sp.tasks to generate implementation tasks
- 🧠 Reflection: Plan addresses all constitution requirements and provides clear implementation path

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All constitution gates passed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate implementation tasks with /sp.tasks