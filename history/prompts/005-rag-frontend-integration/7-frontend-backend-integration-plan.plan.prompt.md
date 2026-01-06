---
id: "7"
title: "Frontend Backend Integration Plan"
stage: "plan"
date: "2025-12-30"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "rag-frontend-integration"
branch: "005-rag-frontend-integration"
user: "ICTech"
command: "sp.plan"
labels: [rag,frontend,backend,docusaurus,integration,chatbot]
links:
  spec: "specs/005-rag-frontend-integration/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "specs/005-rag-frontend-integration/plan.md"
  - "specs/005-rag-frontend-integration/research.md"
  - "specs/005-rag-frontend-integration/data-model.md"
  - "specs/005-rag-frontend-integration/contracts/api-contract.md"
  - "specs/005-rag-frontend-integration/quickstart.md"
tests:
  - "null"
---

## Prompt

Spec-4 Frontend-Backend Integration with FastAPI

- Ensure the frontend UI already exists in physical-ai-book folder (Docusaurus)
- Build chatbot UI integrated into the existing site, displaying the full interface
- Create a new api.py in backend folder and move FastAPI code from agent.py to api.py
- Use agent.py for handling query responses within the new API
- Test end-to-end interaction between frontend UI and backend API

## Response snapshot

Created comprehensive implementation plan for RAG chatbot frontend-backend integration including:

**Phase 0: Research** (`research.md`)
- Backend code structure analysis - splitting `backend/agent.py` into `api.py` (FastAPI endpoints) and `agent.py` (RAG logic)
- Docusaurus component integration patterns - following existing project structure
- React state management - using `useState` for session-only chat history
- FastAPI CORS configuration - allowing requests from `http://localhost:3000`
- Frontend API communication - using browser `fetch` API
- API endpoint contracts - leveraging existing `/api/v1/query` endpoint
- Component testing strategy - manual end-to-end testing for MVP

**Phase 1: Design** (`data-model.md`, `contracts/api-contract.md`, `quickstart.md`)
- Data entities: QueryRequest, QueryResponse, SourceCitation, ChatMessage, ChatbotState
- State transitions and validation rules for both frontend and backend
- Complete API contract with OpenAPI specification documentation
- Quickstart guide with step-by-step setup, testing scenarios, and troubleshooting

**Constitution Check**: PASSED
- All constitutional requirements met
- Authentication explicitly deferred and documented as out-of-scope for MVP

**Project Structure**:
- Backend: `api.py` (FastAPI) + `agent.py` (RAG logic) at root
- Frontend: Chatbot component module in `physical-ai-book/src/components/Chatbot/`
- Service layer: `physical-ai-book/src/services/chatbotApi.ts`

## Outcome

- ✅ Impact: Created complete implementation plan enabling frontend chatbot UI to communicate with backend RAG API, including research findings, data models, API contracts, and quickstart guide
- 🧪 Tests: Not applicable (planning phase)
- 📁 Files: plan.md, research.md, data-model.md, contracts/api-contract.md, quickstart.md
- 🔁 Next prompts: /sp.tasks to generate implementation tasks
- 🧠 Reflection: All clarifications resolved through code analysis and best practices research. No significant architectural decisions requiring ADR - follows established patterns from existing codebase and Docusaurus conventions.

## Evaluation notes (flywheel)

- Failure modes observed: "null"
- Graders run and results (PASS/FAIL): "null"
- Prompt variant (if applicable): "null"
- Next experiment (smallest change to try): "null"
