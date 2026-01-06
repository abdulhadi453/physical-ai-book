# Data Model: RAG Chatbot Frontend Integration

**Feature**: 005-rag-frontend-integration
**Date**: 2025-12-30
**Phase**: 1 - Design

## Overview

This document defines the data entities exchanged between the frontend (React/Docusaurus) and backend (FastAPI) systems, along with frontend state management structures.

---

## Backend Entities (FastAPI/Python)

### QueryRequest

**Purpose**: Request payload from frontend to backend for querying the book.

**Location**: `api.py` (Pydantic model)

**Fields**:
| Field | Type | Required | Validation | Description |
|-------|------|-----------|-------------|-------------|
| `question` | `string` | Yes | min=1, max=1000 | User's natural language question about the book |
| `scope` | `string` (Optional) | No | N/A | Passage identifier to scope retrieval (e.g., "full_book", "passage:{id}") |
| `options` | `QueryOptions` (Optional) | No | N/A | Optional query settings (max_results, temperature) |

**Derived from**: Spec FR-003 (Frontend MUST send user queries to backend API endpoint)

---

### QueryOptions

**Purpose**: Optional settings for query execution.

**Fields**:
| Field | Type | Required | Validation | Description |
|-------|------|-----------|-------------|-------------|
| `max_results` | `integer` | No (default=5) | 1 <= value <= 20 | Number of chunks to retrieve from Qdrant |
| `temperature` | `float` | No (default=0.0) | 0.0 <= value <= 1.0 | Response creativity (0.0 = deterministic, 1.0 = creative) |

---

### QueryResponse

**Purpose**: Response payload from backend to frontend with answer and sources.

**Location**: `api.py` (Pydantic model)

**Fields**:
| Field | Type | Required | Description |
|-------|------|-----------|-------------|
| `success` | `boolean` | Yes | Whether query succeeded |
| `answer` | `string` | Yes | Generated answer from the RAG agent |
| `sources` | `SourceCitation[]` | Yes (can be empty) | List of source chunks used in the answer |
| `latency_ms` | `integer` | Yes | Time taken to generate response in milliseconds |

**Derived from**: Spec FR-004 (display answer text), FR-005 (display source citations)

---

### SourceCitation

**Purpose**: Reference information identifying where in the book the answer came from.

**Fields**:
| Field | Type | Required | Description |
|-------|------|-----------|-------------|
| `chunk_id` | `string` | Yes | Unique identifier for the chunk in Qdrant |
| `section` | `string` | Yes (default="Unknown") | Section title or heading from the book |
| `page` | `integer` (Optional) | No | Page number (if available in chunk metadata) |
| `text_preview` | `string` | Yes (default="") | First 100 characters of chunk for preview |
| `similarity_score` | `float` | Yes | Vector similarity score (0.0 = no match, 1.0 = exact match) |

**Derived from**: Spec Key Entities (Source Citation)

---

### ErrorResponse

**Purpose**: Standard error format returned by backend for API errors.

**Fields**:
| Field | Type | Required | Description |
|-------|------|-----------|-------------|
| `success` | `boolean` | Yes | Always `false` for errors |
| `error` | `string` | Yes | Error type code (e.g., "VALIDATION_ERROR", "RETRIEVAL_ERROR") |
| `message` | `string` | Yes | Human-readable error message |
| `details` | `object` (Optional) | No | Additional error details (if applicable) |

**Derived from**: Spec FR-007 (display user-friendly error message)

---

### HealthResponse

**Purpose**: Health check response for monitoring backend status.

**Fields**:
| Field | Type | Required | Description |
|-------|------|-----------|-------------|
| `status` | `string` | Yes | "healthy" or "degraded" |
| `qdrant_connected` | `boolean` | Yes | Whether Qdrant vector DB is connected |
| `openai_connected` | `boolean` | Yes | Whether OpenAI agent is initialized |
| `timestamp` | `string` | Yes | Response timestamp in ISO 8601 format |

---

## Frontend Entities (React/TypeScript)

### ChatMessage

**Purpose**: Individual message in the chat conversation history.

**Location**: `Chatbot/types.ts` (TypeScript interface)

**Fields**:
```typescript
interface ChatMessage {
  id: string;                    // Unique identifier (UUID)
  type: 'user' | 'assistant';   // Message sender
  content: string;               // Message text (question or answer)
  sources?: SourceCitation[];     // Source citations (only for assistant messages)
  timestamp: Date;               // When the message was created
}
```

**Derived from**: Spec Key Entities (Chat Message, Response Answer)

**Validation**:
- `id` must be unique (use `crypto.randomUUID()` or similar)
- `type` must be 'user' or 'assistant'
- `content` must be non-empty for user messages (FR-009)
- `sources` only populated for assistant messages

---

### ChatbotState

**Purpose**: Complete state of the chatbot component.

**Location**: `Chatbot/index.tsx` (React state)

**State Variables**:
```typescript
interface ChatbotState {
  messages: ChatMessage[];       // Conversation history (FR-008)
  isLoading: boolean;            // Loading state (FR-006)
  error: string | null;         // Error state (FR-007)
  inputValue: string;           // Current input field value
}
```

**State Transitions**:

| Current State | Action | Next State | Description |
|--------------|--------|-------------|-------------|
| `{ messages: [], isLoading: false, error: null, inputValue: "" }` | User types in input | `{ inputValue: "<text>" }` | Input updated without submission |
| `{ messages: [], isLoading: false, error: null, inputValue: "What is Physical AI?" }` | User submits valid input | `{ messages: [...], isLoading: true, error: null, inputValue: "" }` | Request sent, input cleared |
| `{ messages: [...], isLoading: true, ... }` | Backend responds | `{ messages: [...newMessage], isLoading: false, error: null }` | Response displayed |
| `{ messages: [...], isLoading: true, ... }` | Backend error | `{ messages: [...], isLoading: false, error: "message" }` | Error displayed |
| `{ messages: [...], isLoading: false, error: null, ... }` | User clicks clear | `{ messages: [], isLoading: false, error: null, inputValue: "" }` | History cleared (US3) |

**Derived from**: Spec FR-006 (loading indicator), FR-007 (error handling), FR-008 (history), FR-010 (clear mechanism)

---

### SourceCitation (Frontend)

**Purpose**: Frontend representation of backend source citation.

**Location**: `Chatbot/types.ts` (TypeScript interface)

**Fields**:
```typescript
interface SourceCitation {
  chunk_id: string;
  section: string;
  page?: number;
  text_preview: string;
  similarity_score: number;
}
```

**Matches**: Backend `SourceCitation` Pydantic model

**Derived from**: Spec FR-005 (display source citations)

---

## Data Flow Diagram

```
Frontend (React)                    Backend (FastAPI)
┌─────────────────┐                ┌─────────────────┐
│  Chatbot UI    │                │   API Layer    │
│                 │                │    (api.py)    │
│  User Input     │                │                 │
│      ↓          │                │      ↓          │
│  ChatMessage    │                │ QueryRequest   │
│  (type='user') │──────────────→│                │
└─────────────────┘                │      ↓          │
                                  │  RAG Agent     │
┌─────────────────┐                │  (agent.py)    │
│  Chatbot UI    │                │      ↓          │
│                 │                │  Qdrant Search  │
│  Loading State  │                │                 │
│  (isLoading)   │                │                 │
└─────────────────┘                └─────────────────┘
          ↑                                   ↓
          │                            QueryResponse
          │                                   │
          │                            (answer, sources)
          │                                   │
┌─────────────────┐                ┌─────────────────┐
│  Chatbot UI    │                │  Error Handler  │
│                 │←───────────────│                 │
│  ChatMessage    │   Response    │  ErrorResponse  │
│  (type='assistant│                │                 │
│   + sources)   │                └─────────────────┘
└─────────────────┘
```

---

## State Persistence

**Session-Only Storage** (per spec assumption):
- Conversation history stored in React `useState`
- Cleared on page refresh
- No localStorage or database persistence required

**Rationale**:
- Spec FR-003 explicitly states "chat history is cleared on page refresh"
- Out-of-scope items include "Persistent storage of conversation history"
- Simplifies implementation for MVP

---

## Validation Rules

### Frontend Validation (Before API Call)

| Rule | Field | Condition | Error Message |
|------|-------|-----------|---------------|
| V1 | `inputValue` | Empty string or whitespace only | "Please enter a question" |
| V2 | `inputValue` | Length > 1000 characters | "Question must be 1000 characters or less" |

**Derived from**: Spec FR-009 (validate input not empty)

### Backend Validation (Pydantic Models)

| Rule | Field | Condition | HTTP Status |
|------|-------|-----------|--------------|
| BV1 | `question` | Empty or whitespace | 400 Bad Request |
| BV2 | `question` | Length > 1000 | 400 Bad Request |
| BV3 | `options.max_results` | < 1 or > 20 | 422 Unprocessable Entity |
| BV4 | `options.temperature` | < 0.0 or > 1.0 | 422 Unprocessable Entity |

**Derived from**: Backend agent.py validation (T019)

---

## Error Types

| Error Type | HTTP Code | Frontend Display | User Action |
|-----------|-----------|------------------|--------------|
| `VALIDATION_ERROR` | 400 | "Please check your input and try again" | Fix input and resubmit |
| `RETRIEVAL_ERROR` | 500 | "Unable to retrieve book content. Please try again." | Retry later |
| `AGENT_ERROR` | 500 | "Failed to generate response. Please try again." | Retry later |
| `NETWORK_ERROR` | (Frontend) | "Connection failed. Check if backend is running." | Start backend service |

**Derived from**: Spec FR-007 (user-friendly error message), FR-012 (graceful error handling)

---

## Type Mapping: Backend ↔ Frontend

| Backend (Pydantic) | Frontend (TypeScript) | Notes |
|---------------------|----------------------|-------|
| `QueryRequest` | `QueryRequest` | Identical fields |
| `QueryResponse` | `QueryResponse` | Identical fields |
| `SourceCitation` | `SourceCitation` | Identical fields |
| `ErrorResponse` | `ErrorResponse` | Identical fields |
| `HealthResponse` | `HealthResponse` | Identical fields |

**Strategy**: TypeScript interfaces directly mirror Pydantic models to ensure type safety across the boundary.
