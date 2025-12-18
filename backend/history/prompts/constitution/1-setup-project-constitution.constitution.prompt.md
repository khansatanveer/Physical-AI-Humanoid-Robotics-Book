---
id: 1
title: Setup Project Constitution
stage: constitution
date: 2025-12-18
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, setup, project]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 -
---

## Prompt

/sp.constitution

Project: Integrated RAG Chatbot for an AI-Native Technical Book

Description:
Build a production-grade Retrieval-Augmented Generation (RAG) chatbot embedded inside a Docusaurus-published technical book. The chatbot must answer user questions strictly based on the book's content and support a special mode where answers are generated only from user-selected text. The system must be AI-native, reproducible, and aligned with modern LLM engineering best practices.

Audience:
- Technical readers
- AI/ML practitioners
- Software engineers
- Researchers and advanced students

Core Principles:
- Groundedness: All responses must be derived strictly from retrieved content
- Zero Hallucination Policy: If information is not present, explicitly state it
- Determinism & Reproducibility: Same inputs must yield consistent outputs
- Context Isolation: Selected-text queries must not use global knowledge
- Clarity: Responses must be concise, technically accurate, and well-structured
- Transparency: Clearly reference source sections or chapters when applicable

LLM & AI Constraints:
- Language Model: Cohere (no OpenAI APIs allowed)
- Embeddings: Cohere embedding models only
- Prompting must be explicit, minimal, and auditable
- No hidden system prompts or undocumented logic

System Architecture Standards:
- Backend Framework: FastAPI
- Vector Database: Qdrant Cloud (Free Tier)
- Relational Database: Neon Serverless Postgres
- Frontend: Docusaurus with embedded chatbot UI
- RAG Pipeline:
  - Chunking with deterministic rules
  - Metadata-rich embeddings (chapter, section, page)
  - Top-k retrieval with score thresholding

RAG Behavior Rules:
- Answers must be synthesized only from retrieved documents
- For selected-text queries:
  - Ignore all global embeddings
  - Use only the user-provided text as context
- Every response must internally validate source relevance
- If retrieval confidence < threshold → respond with:
  "The requested information is not present in the provided content."

Documentation & Reproducibility:
- All setup steps must be documented
- Environment variables explicitly listed
- One-command local startup
- No proprietary or opaque dependencies

Quality Standards:
- Production-ready code
- Clear module boundaries
- Fully documented API endpoints
- Clean separation of retrieval, generation, and UI layers

Success Criteria:
- Chatbot answers are always grounded in book content
- Selected-text mode never leaks external knowledge
- System can be rebuilt from repository alone
- All architectural decisions are documented and justified

## Response snapshot

I've successfully created the project constitution for your Integrated RAG Chatbot. The constitution now contains all the core principles and constraints you specified:

1. **Core Principles**:
   - Groundedness and Zero Hallucination policy
   - Context Isolation for selected-text vs global queries
   - Determinism & Reproducibility requirements
   - LLM & AI Constraints (Cohere only)
   - Production-Ready Quality standards
   - Transparency and Attribution requirements

2. **Technical Constraints**:
   - Required technology stack (FastAPI, Qdrant Cloud, Neon Postgres, Docusaurus)
   - Security and data handling requirements

3. **Development Workflow**:
   - Code standards and testing requirements

4. **Governance**:
   - Rules for constitution compliance and ADR documentation

The constitution file is now properly set up in `.specify/memory/constitution.md` and reflects all the requirements you specified for the project. This will serve as the foundational document that guides all development decisions for your RAG chatbot project.

## Outcome

- ✅ Impact: Project constitution established with all required principles and constraints
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive project principles
- 🔁 Next prompts: Continue with project implementation based on constitution
- 🧠 Reflection: Constitution provides clear guidance for all future development decisions

## Evaluation notes (flywheel)

- Failure modes observed: N/A
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification based on constitution