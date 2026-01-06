"""
FastAPI application for the RAG chatbot backend.

This module provides the REST API endpoints for the chatbot frontend.
The actual RAG agent logic is in agent.py, using OpenAI Agents SDK with function tools.
"""

import time
import uuid
import logging
from typing import Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

# Import from agent.py
from agent import (
    Config,
    QdrantService,
    RAGAgent,
    QueryRequest,
    QueryResponse,
    SourceCitation,
    ErrorResponse,
    HealthResponse,
    ReadyResponse,
    ValidationError,
    RetrievalError,
    AgentError,
    retrieve_context_chunks,
    extract_sources_from_result
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# ============================================================================
# FastAPI App with CORS
# ============================================================================

app = FastAPI(
    title="Book Q&A Agent API",
    description="RAG agent API for answering questions about the Physical AI book",
    version="1.0.0"
)

# CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
qdrant_service = QdrantService()
rag_agent = RAGAgent(qdrant_service)


@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("Starting Book Q&A Agent...")

    # Validate configuration
    try:
        Config.validate()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration validation failed: {e}")
        raise

    # Connect to Qdrant
    if not qdrant_service.connect():
        logger.warning("Could not connect to Qdrant - some features may not work")

    # Initialize OpenAI Agent
    rag_agent.initialize()


# ============================================================================
# Health & Readiness Endpoints
# ============================================================================

@app.get("/api/v1/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint for monitoring."""
    return HealthResponse(
        status="healthy" if qdrant_service.is_connected() else "degraded",
        qdrant_connected=qdrant_service.is_connected(),
        agent_connected=rag_agent.is_initialized(),
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    )


@app.get("/api/v1/ready", response_model=ReadyResponse)
async def readiness_check():
    """Readiness check for deployment."""
    checks = {
        "qdrant": "ok" if qdrant_service.is_connected() else "error",
        "openai": "ok" if rag_agent.is_initialized() else "error"
    }

    all_ok = all(v == "ok" for v in checks.values())

    return ReadyResponse(
        ready=all_ok,
        checks=checks
    )


# ============================================================================
# Query Endpoint (User Story 1)
# ============================================================================

@app.post("/api/v1/query", response_model=QueryResponse)
async def query_book(request: QueryRequest, http_request: Request):
    """
    Submit a question about the book and receive a grounded answer.

    The agent uses the retrieve_book_content function tool internally to search
    the book and retrieve relevant content before generating an answer.

    Returns the answer with source citations from the retrieved chunks.
    """
    start_time = time.time()

    # Generate request ID for tracing
    request_id = http_request.headers.get("X-Request-ID", str(uuid.uuid4()))

    # Add request ID to logging context
    logger.info(f"[{request_id}] Processing query: {request.question[:50]}...")

    try:
        # Validation for question length
        if not request.question or not request.question.strip():
            raise ValidationError("Question cannot be empty")

        if len(request.question) > 1000:
            raise ValidationError("Question must be 1000 characters or less")

        # Pass question to agent (agent will use retrieve_book_content tool internally)
        logger.info(f"[{request_id}] Calling agent.generate_response...")
        try:
            answer, has_relevant_content, sources_mentioned = await rag_agent.generate_response(request.question)
            logger.info(f"[{request_id}] Agent responded: {len(answer)} chars")
        except Exception as e:
            logger.error(f"[{request_id}] Agent call failed: {e}")
            raise AgentError(f"Agent failed: {str(e)}")

        # Latency tracking
        latency_ms = int((time.time() - start_time) * 1000)

        logger.info(f"[{request_id}] Query completed in {latency_ms}ms")

        # Note: If we want to return the actual sources for the frontend UI components:
        # We can perform a quick retrieval here as well (cached) to populate the sources list
        # or extract them from the agent's run result if we implement a custom Runner.
        # For now, let's do a fast retrieval to populate the UI source citations.
        sources = []
        if has_relevant_content:
            try:
                # This call will be fast due to embedding cache
                results = retrieve_context_chunks(request.question, qdrant_service, limit=3)
                sources = extract_sources_from_result(results)
            except Exception as e:
                logger.warning(f"[{request_id}] Failed to extract citations: {e}")

        return QueryResponse(
            success=True,
            answer=answer,
            sources=sources,
            latency_ms=latency_ms
        )

    except ValidationError as e:
        logger.warning(f"[{request_id}] Validation error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except RetrievalError as e:
        logger.error(f"[{request_id}] Retrieval error: {e}")
        raise HTTPException(status_code=500, detail="Failed to retrieve book content")

    except AgentError as e:
        logger.error(f"[{request_id}] Agent error: {e}")
        raise HTTPException(status_code=500, detail="Failed to generate response")

    except Exception as e:
        import traceback
        logger.error(f"[{request_id}] Unexpected error: {e}")
        logger.error(traceback.format_exc())
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred: {str(e)}")


# ============================================================================
# Error Handlers
# ============================================================================

@app.exception_handler(ValidationError)
async def validation_error_handler(request: Request, exc: ValidationError):
    """Handle validation errors."""
    return JSONResponse(
        status_code=400,
        content=ErrorResponse(
            success=False,
            error="VALIDATION_ERROR",
            message=str(exc)
        ).model_dump()
    )


@app.exception_handler(RetrievalError)
async def retrieval_error_handler(request: Request, exc: RetrievalError):
    """Handle retrieval errors."""
    return JSONResponse(
        status_code=500,
        content=ErrorResponse(
            success=False,
            error="RETRIEVAL_ERROR",
            message="Failed to retrieve book content"
        ).model_dump()
    )


@app.exception_handler(AgentError)
async def agent_error_handler(request: Request, exc: AgentError):
    """Handle agent errors."""
    return JSONResponse(
        status_code=500,
        content=ErrorResponse(
            success=False,
            error="AGENT_ERROR",
            message="Failed to generate response"
        ).model_dump()
    )


# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn

    # Validate configuration on startup
    try:
        Config.validate()
    except ValueError as e:
        print(f"Configuration error: {e}")
        print("Please set the required environment variables in .env")
        exit(1)

    # Run the server
    uvicorn.run(
        "api:app",
        host=Config.APP_HOST,
        port=Config.APP_PORT,
        reload=True,
        log_level=Config.LOG_LEVEL.lower()
    )
