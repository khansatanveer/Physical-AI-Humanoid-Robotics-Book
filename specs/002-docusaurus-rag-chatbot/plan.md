# Implementation Plan: Docusaurus Embedded RAG Chatbot UI

**Branch**: `002-docusaurus-rag-chatbot` | **Date**: 2025-12-17 | **Spec**: [link to spec](../spec.md)
**Input**: Feature specification from `/specs/002-docusaurus-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a clean, embedded chatbot UI in a Docusaurus technical book supporting both global and selected-text queries. The system will connect to a backend API to retrieve grounded responses, with a focus on accessibility, responsiveness, and non-intrusive integration with the reading experience.

## Technical Context

**Language/Version**: JavaScript/TypeScript for frontend, Node.js for Docusaurus
**Primary Dependencies**: React, Docusaurus, Axios/Fetch API
**Storage**: Docusaurus static files, backend API for data retrieval
**Testing**: Jest for frontend, integration tests for API connectivity
**Target Platform**: Web application (Docusaurus documentation site)
**Project Type**: Web (frontend-only with backend API integration)
**Performance Goals**: <200ms UI response, 95% success rate for API queries
**Constraints**: No client-side AI logic, accessible, responsive, non-intrusive
**Scale/Scope**: Supports book readers, students, and engineers with proper source attribution

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, the following requirements must be met:
- ✅ Chatbot must answer questions exclusively from book content (no external knowledge, backend handles this)
- ✅ Technology stack: React for UI components (matches requirements)
- ✅ All content must be clear, well-documented, and reproducible
- ✅ Open source philosophy - using open source tools (React, Docusaurus)
- ✅ Documentation follows Docusaurus standards

## Project Structure

### Documentation (this feature)

```text
specs/002-docusaurus-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
chatbot/
├── components/
│   ├── ChatInterface.jsx          # Main chat UI component with text input and send button
│   ├── Message.jsx                # Individual message display
│   ├── ChatHistory.jsx            # Scrollable chat history container
│   └── ModeSelector.jsx           # Global vs Selected text mode selector
├── hooks/
│   ├── useChat.js                 # Chat state management
│   └── useTextSelection.js        # Text selection handling for selected-text mode
├── services/
│   ├── apiClient.js               # Backend API communication
│   └── utils.js                   # Helper functions
└── styles/
    └── chat.css                   # Chat component styling

src/
├── components/
│   └── DocusaurusChatbot.jsx      # Docusaurus-specific chatbot integration
└── css/
    └── chatbot.css                # Docusaurus-specific styling

.env.example                           # Example environment variables
```

**Structure Decision**: Selected frontend-only structure with API integration. The chatbot UI components are built with React and integrated into Docusaurus, connecting to a backend API for query processing. This provides a clean separation between UI and business logic while maintaining the ability to embed the chatbot directly in the Docusaurus documentation site.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |