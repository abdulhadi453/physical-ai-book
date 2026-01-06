# Quickstart: RAG Chatbot Frontend Integration

**Feature**: 005-rag-frontend-integration
**Date**: 2025-12-30
**Estimated Setup Time**: 15 minutes

## Overview

This guide walks you through setting up and running the chatbot frontend-backend integration locally. It covers:
1. Backend API setup and configuration
2. Frontend chatbot component integration
3. End-to-end testing procedures

---

## Prerequisites

**Software Required**:
- Python 3.10+ for backend
- Node.js 18+ and npm for frontend
- Git for version control

**External Services Required**:
- Qdrant Cloud instance (from feature 002/003)
- OpenAI API key (from feature 004)
- Cohere API key (for embeddings, from feature 003)

**Environment Variables** (backend/.env):
```bash
OPENAI_API_KEY=sk-...              # OpenAI API key
COHERE_API_KEY=...                 # Cohere API key
QDRANT_URL=https://...              # Qdrant Cloud URL
QDRANT_API_KEY=...                # Qdrant Cloud API key
QDRANT_COLLECTION_NAME=book_chunks
APP_HOST=0.0.0.0
APP_PORT=8000
```

---

## Step 1: Backend Setup

### 1.1 Refactor Existing Code

**Current State**: `backend/agent.py` contains both FastAPI app and RAG agent logic.

**Action Required**:
1. Refactor `backend/agent.py` to create `backend/api.py` (FastAPI endpoints)
2. Create `backend/api.py` with FastAPI endpoints, CORS, error handlers
3. Keep RAG agent logic in `backend/agent.py`

**Implementation Notes**:
- `backend/api.py` imports: `from agent import Config, QdrantService, RAGAgent, QueryRequest, QueryResponse, SourceCitation`
- `backend/agent.py` exports: Config, QdrantService, RAGAgent, embedding functions

### 1.2 Add CORS Configuration

In `backend/api.py`, add CORS middleware:

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

### 1.3 Install Dependencies

```bash
cd /path/to/hackathon-book
pip install -r backend/requirements.txt
```

### 1.4 Verify Environment Variables

```bash
# Verify .env file exists
ls backend/.env.example  # Should exist
ls backend/.env           # Should exist (or copy from .env.example)

# If .env doesn't exist, copy from example
cp backend/.env.example backend/.env

# Edit backend/.env and add your API keys
```

### 1.5 Start Backend Service

```bash
# Option 1: Run with Python
python backend/api.py

# Option 2: Run with uvicorn
uvicorn backend.api:app --host 0.0.0.0 --port 8000 --reload
```

**Expected Output**:
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### 1.6 Verify Backend Health

```bash
# Check health endpoint
curl http://localhost:8000/api/v1/health

# Expected response:
# {
#   "status": "healthy",
#   "qdrant_connected": true,
#   "openai_connected": true,
#   "timestamp": "2025-12-30T14:30:00Z"
# }
```

**Troubleshooting**:
- If `qdrant_connected: false`, check `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- If `openai_connected: false`, check `OPENAI_API_KEY` in `.env`
- If `status: "degraded"`, review logs for specific errors

---

## Step 2: Frontend Setup

### 2.1 Navigate to Frontend Directory

```bash
cd /path/to/hackathon-book/physical-ai-book
```

### 2.2 Install Frontend Dependencies

```bash
npm install
```

### 2.3 Create Chatbot Component Structure

Create the following directory structure:

```bash
mkdir -p src/components/Chatbot
mkdir -p src/services
```

Expected file structure:
```
physical-ai-book/
├── src/
│   ├── components/
│   │   └── Chatbot/
│   │       ├── index.tsx              # Main component
│   │       ├── ChatMessage.tsx        # Message display
│   │       ├── ChatInput.tsx         # Input + submit
│   │       ├── LoadingIndicator.tsx  # Loading state
│   │       ├── SourceCitation.tsx    # Citation display
│   │       └── styles.module.css     # Scoped styles
│   └── services/
│       └── chatbotApi.ts            # API service
```

### 2.4 Create API Service (`src/services/chatbotApi.ts`)

```typescript
interface QueryRequest {
  question: string;
  scope?: string;
  options?: {
    max_results?: number;
    temperature?: number;
  };
}

interface SourceCitation {
  chunk_id: string;
  section: string;
  page?: number;
  text_preview: string;
  similarity_score: number;
}

interface QueryResponse {
  success: boolean;
  answer: string;
  sources: SourceCitation[];
  latency_ms: number;
}

interface ErrorResponse {
  success: boolean;
  error: string;
  message: string;
  details?: object;
}

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

export async function checkHealth(): Promise<any> {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);
    if (!response.ok) throw new Error('Health check failed');
    return await response.json();
  } catch (error) {
    throw new Error('Backend is not accessible.');
  }
}
```

### 2.5 Create Chatbot Component (`src/components/Chatbot/index.tsx`)

Refer to the task breakdown in `tasks.md` (generated by `/sp.tasks`) for complete component code.

### 2.6 Add Chatbot to a Page

Edit `docs/intro.md` or any other page to include the chatbot:

```markdown
import Chatbot from '@site/src/components/Chatbot';

# Welcome to Physical AI Book

<Chatbot />

...
```

### 2.7 Start Frontend Development Server

```bash
npm run start
```

**Expected Output**:
```
INFO  Starting the development server...
INFO  Successfully compiled with X modules

You can now view physical-ai-book in the browser.

  Local:            http://localhost:3000
  on Your Network:  http://192.168.x.x:3000
```

---

## Step 3: End-to-End Testing

### 3.1 Navigate to Chatbot Page

Open browser to: `http://localhost:3000/docs/intro` (or whichever page you added the chatbot to)

### 3.2 Test Scenario 1: Valid Question

**Steps**:
1. Enter question: "What is Physical AI?"
2. Click "Submit" or press Enter
3. Wait for response

**Expected Results**:
- User question appears in chat history
- Loading indicator appears (if response >1 second)
- Answer appears with source citations
- Each citation shows: section, page, text preview
- No errors displayed

**Acceptance Criteria**:
- ✅ User sees their question in chat history (FR-008)
- ✅ Loading indicator displays (FR-006)
- ✅ Answer text appears (FR-004)
- ✅ Source citations appear (FR-005)
- ✅ Response time <5 seconds (SC-001)

### 3.3 Test Scenario 2: Empty Input

**Steps**:
1. Leave input field empty
2. Click "Submit"

**Expected Results**:
- Frontend validation error: "Please enter a question"
- No API request is made
- Input field remains empty

**Acceptance Criteria**:
- ✅ Input validation prevents empty submission (FR-009)
- ✅ User-friendly error message displayed (FR-007)

### 3.4 Test Scenario 3: Backend Unavailable

**Steps**:
1. Stop backend: Press Ctrl+C in backend terminal
2. Refresh frontend page
3. Enter a valid question
4. Click "Submit"

**Expected Results**:
- Network error displayed: "Connection failed. Check if backend is running."
- Chatbot does not crash
- User can retry after restarting backend

**Acceptance Criteria**:
- ✅ Error handled gracefully (FR-012)
- ✅ User-friendly error message (FR-007)
- ✅ Interface remains functional (FR-012)

### 3.5 Test Scenario 4: Chat History

**Steps**:
1. Submit 3 different questions
2. Scroll through chat history
3. Verify all question-answer pairs are visible

**Expected Results**:
- All 3 question-answer pairs visible
- Newest messages at bottom
- Scrollable if many messages

**Acceptance Criteria**:
- ✅ History displays up to 20 messages (SC-006)
- ✅ Messages in correct order (FR-008)

### 3.6 Test Scenario 5: Clear Chat

**Steps**:
1. Submit a few questions
2. Click "Clear Chat" button
3. Verify chat interface is empty

**Expected Results**:
- All messages removed
- Input field cleared
- Ready for new questions

**Acceptance Criteria**:
- ✅ History cleared successfully (US3, FR-010)

### 3.7 Test Scenario 6: Consecutive Queries (Success Criterion SC-003)

**Steps**:
1. Submit "What is Physical AI?"
2. Wait for response
3. Submit "What are the core principles?"
4. Wait for response
5. Submit "How does it relate to robotics?"
6. Wait for response

**Expected Results**:
- All 3 queries complete successfully
- Each response includes answer + citations
- No errors or crashes

**Acceptance Criteria**:
- ✅ 3 consecutive cycles complete (SC-003)
- ✅ No interface errors (SC-003)

---

## Step 4: Troubleshooting

### Common Issues

#### Issue: Backend Won't Start

**Symptoms**: `ValueError: OPENAI_API_KEY is required`

**Solution**:
1. Check `backend/.env` file exists
2. Verify all required environment variables are set
3. Ensure `OPENAI_API_KEY`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY` are present

#### Issue: CORS Errors in Browser Console

**Symptoms**: `Access to XMLHttpRequest has been blocked by CORS policy`

**Solution**:
1. Verify CORS middleware is added in `backend/api.py`
2. Check `allow_origins` includes `http://localhost:3000`
3. Restart backend after changes

#### Issue: Frontend Can't Connect to Backend

**Symptoms**: `Connection failed. Check if backend is running.`

**Solution**:
1. Verify backend is running on port 8000
2. Check firewall settings
3. Verify `API_BASE_URL` in `chatbotApi.ts` is correct
4. Try accessing `http://localhost:8000/api/v1/health` in browser

#### Issue: No Source Citations Displayed

**Symptoms**: Answer appears but sources list is empty

**Solution**:
1. Check Qdrant collection has data (`QDRANT_COLLECTION_NAME=book_chunks`)
2. Verify similarity threshold is not too high
3. Check backend logs for retrieval errors
4. Try asking a different question

#### Issue: Chatbot Component Not Visible

**Symptoms**: Page loads but chatbot doesn't appear

**Solution**:
1. Verify component import path is correct: `@site/src/components/Chatbot`
2. Check browser console for errors
3. Ensure component is exported from `index.tsx`
4. Try adding text before the component to verify page renders

---

## Step 5: Development Workflow

### Typical Development Loop

1. **Make Changes**: Edit `api.py`, `agent.py`, or chatbot components
2. **Backend Reloads**: Automatically reloads with `--reload` flag
3. **Frontend Hot Reloads**: Automatically reloads in browser
4. **Test**: Follow end-to-end testing scenarios above
5. **Debug**: Check browser console and backend logs

### Useful Commands

```bash
# Backend
python backend/api.py                        # Start backend with auto-reload
curl http://localhost:8000/api/v1/health     # Health check
curl http://localhost:8000/docs                 # Swagger UI

# Frontend
cd physical-ai-book
npm run start                                # Start dev server
npm run build                                # Production build
npm run serve                                 # Serve production build
```

---

## Step 6: Next Steps

After completing setup and testing:

1. **Explore Features**:
   - Add chatbot to multiple pages
   - Customize chatbot styles
   - Experiment with different queries

2. **Future Enhancements** (out-of-scope for feature 005):
   - Add user authentication (constitution requirement)
   - Persist chat history across sessions
   - Add query suggestions or auto-complete
   - Implement rate limiting
   - Add analytics tracking

3. **Production Deployment**:
   - Update CORS origins for production domain
   - Use environment-specific configs
   - Set up monitoring and logging
   - Configure health checks for load balancer

---

## Additional Resources

- **Backend API Documentation**: `http://localhost:8000/docs` (Swagger UI)
- **API Contract**: [contracts/api-contract.md](./contracts/api-contract.md)
- **Data Model**: [data-model.md](./data-model.md)
- **Feature Specification**: [spec.md](./spec.md)
- **Implementation Plan**: [plan.md](./plan.md)

---

## Support

If you encounter issues not covered in this guide:

1. Check browser console for frontend errors
2. Check backend terminal logs for errors
3. Verify environment variables are correctly set
4. Review API contract for endpoint details
5. Consult data model for expected request/response formats
