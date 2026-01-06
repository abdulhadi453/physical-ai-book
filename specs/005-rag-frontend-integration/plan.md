# Implementation Plan: RAG Chatbot Frontend Integration

**Branch**: `005-rag-frontend-integration` | **Date**: 2025-12-30 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/sp.specify`

## Summary

Build a chatbot UI integrated into the existing Docusaurus website that communicates with a FastAPI backend. The backend (currently in `backend/agent.py`) will be reorganized within the `backend/` folder: `api.py` (FastAPI endpoints) and `agent.py` (RAG agent logic). The frontend chatbot component will send queries to the API endpoint and display responses with source citations. End-to-end testing validates the complete integration.

## Technical Context

**Language/Version**:
- Backend: Python 3.10+
- Frontend: TypeScript 5.x (Docusaurus)

**Primary Dependencies**:
- Backend: FastAPI, Pydantic, Qdrant Client, OpenAI Agents SDK, Cohere, python-dotenv, uvicorn
- Frontend: React 18.x, Docusaurus 3.x, standard browser APIs (fetch)

**Storage**: N/A - Uses existing Qdrant vector database from previous features

**Testing**:
- Backend: pytest for unit tests, manual testing for integration
- Frontend: React Testing Library for component tests, manual end-to-end testing

**Target Platform**:
- Backend: Local development (localhost:8000), Linux server for production
- Frontend: Browser-based web application (any modern browser)

**Project Type**: Web application with frontend (Docusaurus/React) + backend (FastAPI)

**Performance Goals**:
- Response time: <5 seconds for 95% of queries (from spec SC-001)
- Loading indicators: Display for requests >1 second (from spec SC-004)
- History display: Up to 20 messages without degradation (from spec SC-006)

**Constraints**:
- Must use existing backend code (`backend/agent.py`) - no new data ingestion or embedding generation
- Frontend must integrate into existing Docusaurus site in `physical-ai-book/`
- API must communicate locally between services (FR-011)
- CORS must allow frontend requests to backend
- Session-only history - no persistence required
- No new data ingestion or embedding generation

**Scale/Scope**:
- Single-user development environment
- 3 prioritized user stories (P1: submit questions, P2: view history, P3: clear chat)
- 12 functional requirements
- End-to-end testing for local development

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Analysis

| Constitution Requirement | Compliance Status | Notes |
|------------------------|-------------------|-------|
| **Spec-Driven Development** | ✅ PASS | Implementation based on approved spec (specs/005-rag-frontend-integration/spec.md) |
| **AI-Native Integration** | ✅ PASS | Uses existing RAG agent (OpenAI Agents SDK + Qdrant) from feature 004 |
| **Technical Rigor** | ✅ PASS | All responses grounded in retrieved book content with citations |
| **Accessibility** | ✅ PASS | Docusaurus platform provides accessibility; chatbot UI will follow accessible patterns |
| **Safety First** | ✅ PASS | No safety concerns for read-only chatbot interface |
| **RAG Stack** | ✅ PASS | Uses existing FastAPI, Qdrant from feature 004 - no new stack components |
| **Authentication** | ⚠️ DEFERRED | Constitution requires better-auth, but spec explicitly excludes authentication as out-of-scope |
| **Content Chunking** | ✅ PASS | Uses existing chunked content from Qdrant collection |
| **Embedded Chatbot** | ✅ PASS | Chatbot answers strictly from book content only via agent rules |
| **Reusable Intelligence** | ✅ PASS | Uses agent.py for query handling - reusable RAG logic |
| **Verification & Reproducibility** | ✅ PASS | End-to-end testing validates local integration |
| **Ethical & Safety** | ✅ PASS | Read-only book content access, no ethical concerns |
| **Evaluation Alignment** | ✅ PASS | Manual testing validates functional requirements |

### Deferred Requirements

The constitution mandates authentication via better-auth (Section 7). This feature explicitly defers authentication as out-of-scope (see spec "Out of Scope" section). This is justified because:

1. **MVP Focus**: Feature 005 is a proof-of-concept for frontend-backend integration
2. **Incremental Development**: Authentication can be added in a subsequent feature
3. **Local Development**: Current scope is local development environment only

**Justification Required**: A future feature will add authentication to meet constitution requirement.

### Gate Result

**✅ GATE PASSED** - Feature can proceed to Phase 0 research. Authentication is explicitly deferred and documented.

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-frontend-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── api-contract.md  # OpenAPI specification for query endpoint
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Backend: Restructured within backend/ folder
backend/
├── api.py                  # New FastAPI application with endpoints
├── agent.py                # Refactored RAG agent logic (from backend/agent.py)
├── .env.example            # Environment variables template (existing)
└── requirements.txt        # Python dependencies (existing)

# Frontend: Integrated into physical-ai-book/
physical-ai-book/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── Chatbot.tsx              # Main chatbot component
│   │       ├── ChatMessage.tsx          # Individual message display
│   │       ├── ChatInput.tsx           # Input field + submit button
│   │       ├── LoadingIndicator.tsx    # Loading state indicator
│   │       └── SourceCitation.tsx     # Citation display component
│   ├── services/
│   │   └── chatbotApi.ts             # API service for backend communication
│   └── css/
│       └── Chatbot.css               # Chatbot styles
├── docs/
│   └── intro.md                      # Landing page (may add chatbot)
├── docusaurus.config.ts              # Existing Docusaurus config
└── sidebars.ts                      # Existing sidebar config

# Tests (manual for end-to-end validation)
tests/
└── integration/
    └── chatbot-e2e.md              # End-to-end test procedures
```

**Structure Decision**:
- **Backend**: Split `backend/agent.py` into two files within `backend/` folder:
  - `backend/api.py`: FastAPI application with endpoints, CORS, request/response models
  - `backend/agent.py`: RAG agent logic, Qdrant service, embedding functions (refactored from existing file)
- **Frontend**: Create new `Chatbot` component module in existing Docusaurus structure at `physical-ai-book/src/components/Chatbot/`
- **Service Layer**: Frontend API service at `physical-ai-book/src/services/chatbotApi.ts` handles HTTP communication
- **Integration**: Chatbot component can be added to any Docusaurus page via standard React component inclusion

This structure keeps all backend code together in the `backend/` folder while separating API layer from agent logic. The chatbot is a self-contained component that can be added to any page.

## Complexity Tracking

> No constitutional violations requiring justification. All deferred requirements (authentication) are explicitly documented as out-of-scope.

## Implementation Phases Overview

### Phase 0: Research (Complete)
**Output**: `research.md`
- Study existing `backend/agent.py` code structure
- Research Docusaurus component integration patterns
- Research React state management for chat history
- Document FastAPI CORS configuration requirements
- Define API endpoint contracts

### Phase 1: Design (Complete)
**Outputs**:
- `data-model.md` - Frontend/backend data entities
- `contracts/api-contract.md` - OpenAPI specification
- `quickstart.md` - Development and testing guide

### Phase 2: Implementation (Tasks - via `/sp.tasks`)
**Not created by `/sp.plan`** - Will be generated by `/sp.tasks` command

### Phase 3: Testing & Validation (Tasks - via `/sp.tasks`)
**Not created by `/sp.plan`** - Will be generated by `/sp.tasks` command
