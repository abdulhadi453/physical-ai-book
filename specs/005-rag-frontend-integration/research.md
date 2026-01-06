# Research: RAG Chatbot Frontend Integration

**Feature**: 005-rag-frontend-integration
**Date**: 2025-12-30
**Phase**: 0 - Outline & Research

## Overview

This document consolidates research findings for integrating a chatbot UI with the existing RAG backend. All clarifications from the plan's technical context have been resolved through code analysis and best practices research.

---

## Research Findings

### R1: Backend Code Structure (backend/agent.py)

**Question**: What is the current structure of `backend/agent.py` and how should it be refactored?

**Analysis**:
- Current file contains 724 lines combining:
  - FastAPI application (`app = FastAPI(...)`)
  - Qdrant service class (`QdrantService`)
  - RAG agent class (`RAGAgent`)
  - Pydantic models (request/response schemas)
  - Helper functions (embeddings, context formatting)
  - Endpoints: `/api/v1/query`, `/api/v1/health`, `/api/v1/ready`

**Key Classes**:
- `Config`: Environment variable configuration
- `QdrantService`: Vector database operations
- `RAGAgent`: OpenAI Agent with retrieval tool
- `QueryRequest`, `QueryResponse`, `SourceCitation`: Request/response models

**Dependencies**:
- FastAPI, Pydantic, Qdrant Client, OpenAI Agents SDK, Cohere (embeddings), python-dotenv, uvicorn

**Decision**: Split into two files within `backend/` folder:
- **`backend/api.py`**: FastAPI app, endpoints, CORS, error handlers, Pydantic models
- **`backend/agent.py`**: `Config`, `QdrantService`, `RAGAgent`, embedding functions

**Rationale**:
- Separates API layer from business logic (agent processing)
- Keeps all backend code together in one folder
- Makes agent logic reusable for other services
- Follows single responsibility principle
- Allows independent testing of API and agent components

**Alternatives Considered**:
1. Keep single file - Rejected: Harder to test, violates SRP
2. Move to root level (api.py + agent.py) - Rejected: User wants all backend code in backend/ folder
3. Split into 3 files (api.py, agent.py, models.py) - Rejected: Models naturally belong with API layer

---

### R2: Docusaurus Component Integration Patterns

**Question**: How to integrate a new React component into an existing Docusaurus site?

**Analysis**:
- Docusaurus 3.x uses standard React components
- Components should be placed in `src/components/`
- Component directories follow Docusaurus conventions: `ComponentName/index.tsx` pattern
- Components can be imported and used in any page or other component
- Styling: Use `className` props and `module.css` or global CSS

**Existing Components** (found in `physical-ai-book/src/components/`):
- `ConceptCard/`, `ExerciseBox/`, `HomepageFeatures/`, `PrerequisiteIndicator/`, `ResourceLink/`, `SummarySection/`

**Decision**: Create `Chatbot/` component directory following existing patterns:
- `Chatbot/index.tsx` - Main component
- `Chatbot/ChatMessage.tsx` - Individual message display
- `Chatbot/ChatInput.tsx` - Input field + submit button
- `Chatbot/LoadingIndicator.tsx` - Loading state
- `Chatbot/SourceCitation.tsx` - Citation display
- `Chatbot/styles.module.css` - Scoped styles

**Rationale**:
- Matches existing Docusaurus project structure
- Follows React best practices (component composition)
- Scoped CSS prevents style conflicts
- Can be added to any page via `import Chatbot from '@site/src/components/Chatbot'`

**Alternatives Considered**:
1. Use external UI library (Material UI, Chakra) - Rejected: Adds dependency, over-engineering for MVP
2. Build single monolithic component - Rejected: Harder to test and maintain
3. Use Docusaurus Swizzle API - Rejected: Intended for theme customization, not new features

---

### R3: React State Management for Chat History

**Question**: What is the best approach for managing chat history in React?

**Analysis**:
- Chat history is session-only (no persistence required per spec)
- Standard React state management options:
  - `useState` - Built-in, simple, no extra dependencies
  - `useReducer` - Better for complex state transitions
  - Context API - For sharing state across components
  - External libraries (Redux, Zustand, Jotai) - Overkill for single-component state

**State Requirements**:
- Array of messages (user questions + agent responses)
- Loading state (boolean)
- Error state (string/null)
- Clear history operation

**Decision**: Use `useState` with typed interfaces:
```typescript
interface ChatMessage {
  id: string;
  type: 'user' | 'assistant';
  content: string;
  sources?: SourceCitation[];
  timestamp: Date;
}

const [messages, setMessages] = useState<ChatMessage[]>([]);
const [isLoading, setIsLoading] = useState(false);
const [error, setError] = useState<string | null>(null);
```

**Rationale**:
- Simple and built into React (no extra dependencies)
- Meets all state requirements for chat history
- Easy to understand and maintain
- Standard pattern for component-level state

**Alternatives Considered**:
1. `useReducer` - Rejected: State transitions are simple (add message, set loading, set error)
2. Context API - Rejected: State doesn't need to be shared across multiple components
3. External library - Rejected: Over-engineering for single-component use case

---

### R4: FastAPI CORS Configuration

**Question**: How to configure CORS for local frontend-backend communication?

**Analysis**:
- Docusaurus dev server runs on `http://localhost:3000` (default)
- Backend API will run on `http://localhost:8000` (from existing `Config.APP_PORT`)
- Cross-origin requests require CORS configuration
- FastAPI provides `CORSMiddleware` for this purpose

**CORS Configuration Requirements**:
- Allow requests from `http://localhost:3000` (frontend dev server)
- Allow `GET`, `POST`, `OPTIONS` methods
- Allow standard headers (`Content-Type`, `Authorization` if added later)
- Support credentials if needed (cookies, auth headers)

**Decision**: Add `CORSMiddleware` to FastAPI app:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Docusaurus dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Rationale**:
- Standard FastAPI pattern for CORS
- Allows development without CORS errors
- Can be configured for production origins later
- Follows FastAPI documentation best practices

**Alternatives Considered**:
1. Use proxy configuration in Docusaurus - Rejected: Adds complexity, requires dev server config
2. Disable CORS entirely (allow all origins) - Rejected: Security risk in production
3. Backend on same origin (same port) - Rejected: Requires build integration, overkill for local dev

---

### R5: Frontend API Communication Patterns

**Question**: What is the best pattern for frontend-to-backend HTTP communication?

**Analysis**:
- Browser `fetch` API is built-in and modern
- Axios provides better error handling, request/response interceptors
- React Query (TanStack Query) provides caching, loading states, error states

**Requirements**:
- POST requests to `/api/v1/query` endpoint
- Handle loading state
- Display error messages
- Parse JSON responses

**Decision**: Create `chatbotApi.ts` service with `fetch` API:
```typescript
interface QueryRequest {
  question: string;
  scope?: string;
  options?: {
    max_results?: number;
    temperature?: number;
  };
}

interface QueryResponse {
  success: boolean;
  answer: string;
  sources: SourceCitation[];
  latency_ms: number;
}

export async function queryBook(question: string): Promise<QueryResponse> {
  const response = await fetch('http://localhost:8000/api/v1/query', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ question }),
  });
  // ... error handling and JSON parsing
}
```

**Rationale**:
- Built-in browser API (no extra dependencies)
- Simple and straightforward
- Meets all requirements without over-engineering
- Can be upgraded to Axios or React Query later if needed

**Alternatives Considered**:
1. Axios library - Rejected: Adds 100KB+ dependency for simple use case
2. React Query - Rejected: Overkill for MVP, caching not required yet
3. GraphQL client - Rejected: Backend is REST API, no GraphQL schema

---

### R6: API Endpoint Contracts

**Question**: What is the API contract between frontend and backend?

**Analysis**:
- Existing backend has `POST /api/v1/query` endpoint
- Request model: `QueryRequest` with `question`, `scope`, `options`
- Response model: `QueryResponse` with `success`, `answer`, `sources`, `latency_ms`
- Sources model: `SourceCitation` with `chunk_id`, `section`, `page`, `text_preview`, `similarity_score`

**Decision**: Use existing contract with minimal changes:
- Frontend TypeScript interfaces mirror backend Pydantic models
- No changes to backend request/response structure
- Frontend only uses `question` field initially (scope, options optional)

**Rationale**:
- Leverages existing tested backend code
- Minimal refactoring required
- Backend contract is already documented in `backend/agent.py`
- Optional fields (scope, options) can be added later if needed

**Alternatives Considered**:
1. Create new simplified endpoint - Rejected: Duplicates existing work
2. Modify existing endpoint to remove optional fields - Rejected: Loses flexibility for future features
3. Use GraphQL schema - Rejected: Requires backend rewrite, over-engineering

---

### R7: Component Testing Strategy

**Question**: How to test the chatbot frontend components?

**Analysis**:
- React Testing Library is Docusaurus default
- Docusaurus provides `@site` alias for importing components
- Testing requires rendering components and verifying behavior

**Decision**: Manual end-to-end testing for MVP:
1. Start backend: `python api.py`
2. Start frontend: `npm run start` in `physical-ai-book/`
3. Open browser to `http://localhost:3000`
4. Test scenarios from spec:
   - Submit valid question
   - Verify answer displays with citations
   - Verify loading indicator
   - Test empty input validation
   - Test backend unavailable scenario

**Rationale**:
- Fast feedback for MVP
- Tests actual user experience (not just unit tests)
- No test infrastructure setup required
- Aligns with spec success criteria (SC-003: 3 consecutive question-answer cycles)

**Alternatives Considered**:
1. React Testing Library unit tests - Rejected: Time-consuming, integration testing more valuable for MVP
2. Playwright/Cypress E2E tests - Rejected: Over-engineering for single-component MVP
3. No testing at all - Rejected: Violates spec requirement for validation

---

## Summary of Decisions

| Decision | Chosen Approach | Rationale |
|----------|----------------|------------|
| Backend structure | Split `api.py` + `agent.py` | Separation of concerns, testability |
| Frontend components | `Chatbot/` module in `physical-ai-book/src/components/` | Follows Docusaurus patterns |
| State management | React `useState` | Simple, built-in, no dependencies |
| CORS config | FastAPI `CORSMiddleware` | Standard pattern, dev-friendly |
| API communication | Browser `fetch` API | Built-in, no dependencies |
| API contract | Existing `POST /api/v1/query` | Leverages tested backend |
| Testing strategy | Manual E2E testing | Fast feedback, tests actual UX |

---

## Next Steps (Phase 1: Design)

1. Generate `data-model.md` with frontend/backend entity definitions
2. Generate `contracts/api-contract.md` with OpenAPI specification
3. Generate `quickstart.md` with development and testing procedures
4. Update agent context with new technologies (React, Docusaurus integration)
