# API Contract: RAG Chatbot Backend

**Feature**: 005-rag-frontend-integration
**Date**: 2025-12-30
**Base URL**: `http://localhost:8000`
**API Version**: `v1`

## Overview

This document defines the REST API contract between the chatbot frontend and the RAG backend. All endpoints return JSON responses with consistent error handling.

---

## Authentication

**Status**: Not implemented (out-of-scope for feature 005)

Future implementations will use authentication via `better-auth` (per constitution requirement).

---

## Common Headers

| Header | Type | Required | Description |
|--------|------|-----------|-------------|
| `Content-Type` | `string` | Yes | Must be `application/json` |
| `X-Request-ID` | `string` (UUID) | No | Request identifier for tracing (auto-generated if not provided) |

---

## Endpoints

### 1. Query Book

**Endpoint**: `POST /api/v1/query`

**Description**: Submit a question about the Physical AI book and receive a grounded answer with source citations.

**Request Body**:
```json
{
  "question": "What is Physical AI?",
  "scope": "full_book",           // Optional: "full_book" or "passage:{id}"
  "options": {                      // Optional
    "max_results": 5,              // Default: 5, Range: 1-20
    "temperature": 0.0             // Default: 0.0, Range: 0.0-1.0
  }
}
```

**Request Schema**:
| Field | Type | Required | Constraints | Description |
|-------|------|-----------|-------------|-------------|
| `question` | `string` | Yes | 1-1000 chars | User's natural language question |
| `scope` | `string` | No | N/A | Passage identifier to scope retrieval |
| `options` | `object` | No | N/A | Optional query settings |
| `options.max_results` | `integer` | No | 1-20 | Number of chunks to retrieve |
| `options.temperature` | `float` | No | 0.0-1.0 | Response creativity |

**Success Response** (`200 OK`):
```json
{
  "success": true,
  "answer": "Physical AI refers to artificial intelligence systems designed to interact with and operate in physical environments...",
  "sources": [
    {
      "chunk_id": "abc123",
      "section": "Chapter 1: Introduction",
      "page": 12,
      "text_preview": "Physical AI represents the convergence of artificial intelligence...",
      "similarity_score": 0.92
    },
    {
      "chunk_id": "def456",
      "section": "Chapter 2: Fundamentals",
      "page": 25,
      "text_preview": "The core principles of Physical AI include perception, reasoning...",
      "similarity_score": 0.87
    }
  ],
  "latency_ms": 2430
}
```

**Response Schema**:
| Field | Type | Description |
|-------|------|-------------|
| `success` | `boolean` | Always `true` for successful responses |
| `answer` | `string` | Generated answer from RAG agent |
| `sources` | `SourceCitation[]` | List of source chunks (can be empty) |
| `latency_ms` | `integer` | Time taken in milliseconds |

**Error Responses**:

| HTTP Code | Error Type | Message |
|-----------|-------------|---------|
| `400 Bad Request` | `VALIDATION_ERROR` | Question cannot be empty / Question must be 1000 characters or less |
| `500 Internal Server Error` | `RETRIEVAL_ERROR` | Failed to retrieve book content |
| `500 Internal Server Error` | `AGENT_ERROR` | Failed to generate response |
| `500 Internal Server Error` | (Generic) | An unexpected error occurred |

**Example Error Response**:
```json
{
  "success": false,
  "error": "VALIDATION_ERROR",
  "message": "Question cannot be empty",
  "details": null
}
```

**Derived from**: Spec FR-003 (send queries), FR-004 (display answer), FR-005 (display sources)

---

### 2. Health Check

**Endpoint**: `GET /api/v1/health`

**Description**: Check if the backend service is healthy and connected to dependencies.

**Request Body**: None

**Success Response** (`200 OK`):
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "openai_connected": true,
  "timestamp": "2025-12-30T14:30:00Z"
}
```

**Response Schema**:
| Field | Type | Description |
|-------|------|-------------|
| `status` | `string` | `"healthy"` or `"degraded"` |
| `qdrant_connected` | `boolean` | Qdrant vector DB connection status |
| `openai_connected` | `boolean` | OpenAI agent initialization status |
| `timestamp` | `string` | ISO 8601 formatted timestamp |

---

### 3. Readiness Check

**Endpoint**: `GET /api/v1/ready`

**Description**: Check if the backend is ready to serve requests (all dependencies connected).

**Request Body**: None

**Success Response** (`200 OK`):
```json
{
  "ready": true,
  "checks": {
    "qdrant": "ok",
    "openai": "ok"
  }
}
```

**Response Schema**:
| Field | Type | Description |
|-------|------|-------------|
| `ready` | `boolean` | Service is ready if all checks are `"ok"` |
| `checks` | `object` | Individual dependency statuses |
| `checks.qdrant` | `string` | `"ok"` or `"error"` |
| `checks.openai` | `string` | `"ok"` or `"error"` |

---

## CORS Configuration

The backend API includes CORS middleware to allow requests from the frontend:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Docusaurus dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Allowed Origins**:
- Development: `http://localhost:3000` (Docusaurus dev server)
- Production: (To be configured based on deployment)

---

## Rate Limiting

**Status**: Not implemented (out-of-scope for feature 005)

Future implementations may include rate limiting to prevent abuse.

---

## Error Handling Strategy

| Scenario | HTTP Code | Error Type | User Message |
|----------|-----------|-------------|--------------|
| Empty question | 400 | `VALIDATION_ERROR` | "Please enter a question" |
| Question too long (>1000 chars) | 400 | `VALIDATION_ERROR` | "Question must be 1000 characters or less" |
| Qdrant connection failure | 500 | `RETRIEVAL_ERROR` | "Unable to retrieve book content. Please try again." |
| OpenAI agent failure | 500 | `AGENT_ERROR` | "Failed to generate response. Please try again." |
| Network timeout | 500 | (Generic) | "Request timed out. Please try again." |
| Unhandled exception | 500 | (Generic) | "An unexpected error occurred. Please try again." |

**Derived from**: Spec FR-007 (user-friendly error messages), FR-012 (graceful error handling)

---

## Performance Expectations

| Metric | Target | Description |
|--------|---------|-------------|
| p95 Response Time | <5 seconds | 95% of queries complete within 5 seconds (SC-001) |
| p99 Response Time | <10 seconds | 99% of queries complete within 10 seconds |
| Timeout | 30 seconds | Maximum wait time for query completion |
| Max Chunks Retrieved | 20 | Maximum number of chunks retrieved per query |

---

## Versioning Strategy

**Current Version**: `v1`

**Versioning Approach**:
- URL path includes version: `/api/v1/...`
- Breaking changes will increment version (e.g., `/api/v2/...`)
- Non-breaking changes maintain same version
- Deprecated versions will be announced and maintained for 6 months

---

## OpenAPI/Swagger Specification

The FastAPI app generates automatic OpenAPI documentation at:

**URL**: `http://localhost:8000/docs` (Swagger UI)

This provides interactive API testing and complete schema documentation.

---

## Frontend Integration Example

```typescript
// chatbotApi.ts
const API_BASE_URL = 'http://localhost:8000/api/v1';

export async function queryBook(question: string): Promise<QueryResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ question }),
    });

    if (!response.ok) {
      const error: ErrorResponse = await response.json();
      throw new Error(error.message || 'Request failed');
    }

    return await response.json();
  } catch (error) {
    // Handle network errors
    throw new Error('Connection failed. Check if backend is running.');
  }
}
```

---

## Testing Endpoints

### Manual Testing with curl

```bash
# Health check
curl http://localhost:8000/api/v1/health

# Query endpoint
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'

# With options
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the core principles?",
    "options": {
      "max_results": 10,
      "temperature": 0.3
    }
  }'
```

### Testing with Swagger UI

Navigate to `http://localhost:8000/docs` in a browser to interactively test all endpoints.

---

## Implementation Notes

1. **Question Length Limit**: The backend enforces a 1000 character limit on questions. Frontend should enforce this client-side for better UX.

2. **Source Citations**: Responses always include a `sources` array. It may be empty if no relevant content was found.

3. **Latency Tracking**: All responses include `latency_ms` for monitoring and performance analysis.

4. **Request IDs**: Requests can include `X-Request-ID` header for tracing. If omitted, a UUID is auto-generated.

5. **Async Processing**: The `/query` endpoint is async and may take up to 30 seconds to complete. Frontend should show a loading indicator for requests >1 second (SC-004).

6. **Error Recovery**: All errors return structured `ErrorResponse` with `error` type code and `message`. Frontend should display user-friendly messages based on `message` field.
