# Agent-Based RAG Backend

# A RAG agent that answers questions about the Physical AI book using stored
# embeddings from Qdrant, powered by OpenAI Agents SDK.
#
# This module provides core agent logic with function tools for retrieval.
# Use api.py for FastAPI REST API endpoints.

import os
import logging
from typing import Optional

from pathlib import Path
from dotenv import load_dotenv
from pydantic import BaseModel, Field
from qdrant_client import QdrantClient
import cohere

# Load environment variables from .env or .env.example
env_path = Path(__file__).parent / ".env"
if not env_path.exists():
    env_path = Path(__file__).parent / ".env.example"
load_dotenv(dotenv_path=env_path)

# OpenAI Agents SDK
try:
    from agents import (
        Agent,
        Runner,
        set_default_openai_key,
        function_tool,
        OpenAIChatCompletionsModel,
        set_tracing_disabled,
        AsyncOpenAI
    )
    OPENAI_AGENTS_AVAILABLE = True
except ImportError:
    OPENAI_AGENTS_AVAILABLE = False
    print("Warning: OpenAI Agents SDK not installed. Run: pip install openai-agents")

# Disable tracing to avoid errors
try:
    set_tracing_disabled(disabled=True)
except:
    pass

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# ============================================================================
# Configuration (T005)
# ============================================================================

class Config:
    """Application configuration loaded from environment variables."""

    # OpenRouter (for Agents SDK via OpenRouter)
    OPENROUTER_API_KEY: str = os.getenv("OPENROUTER_API_KEY", "")

    # Cohere (for embeddings - matches Qdrant collection)
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY", None)
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

    # Application
    APP_HOST: str = os.getenv("APP_HOST", "0.0.0.0")
    APP_PORT: int = int(os.getenv("APP_PORT", "8000"))
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "info")

    # Query Settings
    MAX_RETRIEVAL_CHUNKS: int = int(os.getenv("MAX_RETRIEVAL_CHUNKS", "10"))
    SIMILARITY_THRESHOLD: float = float(os.getenv("SIMILARITY_THRESHOLD", "0.1"))  # More permissive for better recall
    QUERY_TIMEOUT_SECONDS: int = int(os.getenv("QUERY_TIMEOUT_SECONDS", "30"))

    # Model Settings
    MODEL_NAME: str = os.getenv("MODEL_NAME", "mistralai/devstral-2512:free")

    @classmethod
    def validate(cls) -> bool:
        """Validate that required configuration is present."""
        if not cls.OPENROUTER_API_KEY:
            raise ValueError("OPENROUTER_API_KEY is required")
        if not cls.QDRANT_URL:
            raise ValueError("QDRANT_URL is required")
        return True


# ============================================================================
# Custom Exceptions (T012)
# ============================================================================

class RetrievalError(Exception):
    """Raised when Qdrant retrieval fails."""
    pass


class AgentError(Exception):
    """Raised when OpenAI agent processing fails."""
    pass


class ValidationError(Exception):
    """Raised when request validation fails."""
    pass


# ============================================================================
# Pydantic Models (T006-T009)
# ============================================================================

class QueryOptions(BaseModel):
    """Optional settings for query execution."""
    max_results: int = Field(default=5, ge=1, le=20, description="Number of chunks to retrieve")
    temperature: float = Field(default=0.0, ge=0.0, le=1.0, description="Response creativity (0.0-1.0)")


class QueryRequest(BaseModel):
    """Input from the chatbot UI to query the book."""
    question: str = Field(..., min_length=1, max_length=1000, description="Natural language question about the book")
    scope: Optional[str] = Field(default=None, description="Optional passage identifier to scope retrieval")
    options: Optional[QueryOptions] = Field(default=None, description="Optional query settings")


class SourceCitation(BaseModel):
    """Reference to a source chunk used in the response."""
    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    section: str = Field(default="Unknown", description="Section title or heading")
    page: Optional[int] = Field(default=None, description="Page number")
    text_preview: str = Field(default="", description="First 100 chars of chunk for preview")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Vector similarity score")


class QueryResponse(BaseModel):
    """Output returned to the chatbot UI with answer and sources."""
    success: bool = Field(..., description="Whether query succeeded")
    answer: str = Field(..., description="Generated answer from the agent")
    sources: list[SourceCitation] = Field(default_factory=list, description="List of source chunks used")
    latency_ms: int = Field(..., description="Time taken to generate response")


class ErrorResponse(BaseModel):
    """Standard error format for API errors."""
    success: bool = Field(default=False, description="Always false for errors")
    error: str = Field(..., description="Error type code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[dict] = Field(default=None, description="Additional error details")


class HealthResponse(BaseModel):
    """Health check response."""
    status: str = Field(..., description="Health status")
    qdrant_connected: bool = Field(..., description="Qdrant connection status")
    agent_connected: bool = Field(..., description="Agent connection status")
    timestamp: str = Field(..., description="Response timestamp")


class ReadyResponse(BaseModel):
    """Readiness check response."""
    ready: bool = Field(..., description="Whether service is ready")
    checks: dict = Field(..., description="Individual check statuses")


# ============================================================================
# Qdrant Client (T010)
# ============================================================================

class QdrantService:
    """Service for interacting with Qdrant vector database."""

    def __init__(self):
        self.client: Optional[QdrantClient] = None
        self._connected = False

    def connect(self) -> bool:
        """Establish connection to Qdrant."""
        try:
            self.client = QdrantClient(
                url=Config.QDRANT_URL,
                api_key=Config.QDRANT_API_KEY
            )
            # Test connection by getting collection info
            self.client.get_collection(collection_name=Config.QDRANT_COLLECTION_NAME)
            self._connected = True
            logger.info(f"Connected to Qdrant at {Config.QDRANT_URL}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            self._connected = False
            return False

    def is_connected(self) -> bool:
        """Check if connected to Qdrant."""
        return self._connected

    def search(
        self,
        query_vector: list[float],
        limit: int = 5,
        score_threshold: Optional[float] = None,
        scope_filter: Optional[str] = None
    ) -> list[dict]:
        """
        Search for similar chunks in Qdrant.

        Args:
            query_vector: The embedding vector of the query
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score (0.0-1.0)
            scope_filter: Optional scope filter (passage ID)

        Returns:
            List of search results with metadata
        """
        if not self._connected or self.client is None:
            raise RetrievalError("Not connected to Qdrant")

        try:
            query_response = self.client.query_points(
                collection_name=Config.QDRANT_COLLECTION_NAME,
                query=query_vector,
                limit=limit,
                score_threshold=score_threshold or Config.SIMILARITY_THRESHOLD
            )

            # Format results - query_response.points contains the hits
            formatted_results = []
            for point in query_response.points:
                # Map payload fields (Match with actual Qdrant structure: 'content', 'title', 'url')
                content = point.payload.get("content", point.payload.get("text", ""))
                section = point.payload.get("title", "Unknown Section")

                formatted_results.append({
                    "id": point.id,
                    "score": point.score,
                    "text": content,
                    "metadata": {
                        "section": section,
                        "page": point.payload.get("page"), # Keep if exists
                        "url": point.payload.get("url", "")
                    }
                })

            return formatted_results

        except Exception as e:
            logger.error(f"Qdrant search failed: {e}")
            raise RetrievalError(f"Search failed: {e}")


# ============================================================================
# OpenAI Client for Embeddings (higher rate limits than Cohere)
# ============================================================================

# Global Cohere client for embeddings
cohere_client: Optional[cohere.Client] = None
_embedding_cache: dict[str, list[float]] = {}  # Simple LRU cache to reduce API calls


def get_embedding(text: str) -> list[float]:
    """
    Get embedding vector for text using Cohere's embed-english-v3.0 model.
    Includes aggressive retry logic for trial API keys (429 handling).

    Args:
        text: The text to embed

    Returns:
        Embedding vector as list of floats (1024 dimensions)
    """
    global cohere_client, _embedding_cache

    # Check cache first
    cache_key = text.lower().strip()
    if cache_key in _embedding_cache:
        logger.info(f"Using cached embedding for: {text[:30]}...")
        return _embedding_cache[cache_key]

    # Initialize client if needed
    if cohere_client is None:
        cohere_client = cohere.Client(api_key=Config.COHERE_API_KEY)

    import time
    max_retries = 5
    # Aggressive backoff for trial keys
    # Trial keys often have a limit of 5-10 calls per minute
    delays = [2, 5, 10, 20, 30]

    for attempt in range(max_retries):
        try:
            logger.info(f"Requesting Cohere embedding (Attempt {attempt + 1}/{max_retries})...")
            response = cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",
                input_type="search_query" # Specifically for RAG queries
            )
            embedding = response.embeddings[0]

            # Cache the result
            _embedding_cache[cache_key] = embedding
            logger.info(f"Generated Cohere embedding for: {text[:30]}... (dim: {len(embedding)})")

            return embedding

        except Exception as e:
            error_msg = str(e).lower()
            if "rate" in error_msg or "429" in error_msg:
                if attempt < max_retries - 1:
                    delay = delays[attempt]
                    logger.warning(f"Cohere rate limited (429). Waiting {delay}s before retry...")
                    time.sleep(delay)
                    continue
                else:
                    logger.error(f"Cohere API rate limit exceeded after {max_retries} retries")
                    raise RetrievalError("Cohere API rate limit exceeded. Please try again in 1 minute.")
            else:
                logger.error(f"Cohere embedding failed: {e}")
                raise RetrievalError(f"Embedding failed: {e}")

    return [0.0] * 1024


# ============================================================================
# Retrieval Tool using @function_tool (T011, T016)
# ============================================================================

from typing import Annotated


class RetrievedChunk(BaseModel):
    """Single retrieved chunk from the book."""
    text: str = Field(description="The content of the chunk")
    section: str = Field(description="The section title")
    page: Optional[int] = Field(default=None, description="Page number")
    score: float = Field(description="Similarity score")


class RetrievalResult(BaseModel):
    """Result of the retrieval operation."""
    chunks: list[RetrievedChunk] = Field(description="List of retrieved chunks")
    total_found: int = Field(description="Total number of chunks found")


# Global reference to Qdrant service for the tool
_qdrant_service_ref: Optional[QdrantService] = None


def set_qdrant_service(service: QdrantService):
    """Set the global Qdrant service reference for the tool."""
    global _qdrant_service_ref
    _qdrant_service_ref = service


@function_tool
def retrieve_book_content(
    question: Annotated[str, "The user's question to search for in the book"],
    max_results: Annotated[int, "Maximum number of chunks to retrieve (default 5, max 10)"] = 5
) -> RetrievalResult:
    """
    Retrieve relevant content from the Physical AI book based on the user's question.

    This tool performs semantic search in the book's content to find the most relevant
    passages. Use this tool whenever you need information about the book's content.
    """
    global _qdrant_service_ref

    if _qdrant_service_ref is None:
        logger.error("Qdrant service not initialized")
        return RetrievalResult(chunks=[], total_found=0)

    try:
        # Encode the query
        query_vector = encode_query(question)

        # Search in Qdrant
        results = _qdrant_service_ref.search(
            query_vector=query_vector,
            limit=min(max_results, 10),
            score_threshold=Config.SIMILARITY_THRESHOLD
        )

        # Convert to structured format
        retrieved_chunks = []
        for result in results:
            metadata = result.get("metadata", {})
            retrieved_chunks.append(RetrievedChunk(
                text=result.get("text", ""),
                section=metadata.get("section", "Unknown"),
                page=metadata.get("page"),
                score=result.get("score", 0.0)
            ))

        logger.info(f"Retrieved {len(retrieved_chunks)} chunks for question: {question[:50]}...")

        # If empty, return a pseudo-chunk to tell the LLM that retrieval was empty
        if not retrieved_chunks:
            return RetrievalResult(
                chunks=[RetrievedChunk(
                    text="NO RELEVANT CONTENT FOUND IN THE BOOK. The user's question might not be covered in the Physical AI book sections indexed in the database.",
                    section="SYSTEM",
                    page=0,
                    score=0.0
                )],
                total_found=0
            )

        return RetrievalResult(
            chunks=retrieved_chunks,
            total_found=len(retrieved_chunks)
        )

    except Exception as e:
        logger.error(f"Retrieval failed: {e}")
        return RetrievalResult(chunks=[], total_found=0)


# ============================================================================
# OpenAI Agent (T011, T016)
# ============================================================================

class RAGAgent:
    """OpenAI Agent with retrieval capabilities using OpenAI Agents SDK."""

    def __init__(self, qdrant_service: QdrantService):
        self.qdrant_service = qdrant_service
        self.agent: Optional[Agent] = None
        self._initialized = False

        # Set the global service reference for the tool
        set_qdrant_service(qdrant_service)

    def initialize(self) -> bool:
        """Initialize the OpenAI Agent with retrieval tool."""
        if not OPENAI_AGENTS_AVAILABLE:
            logger.warning("OpenAI Agents SDK not available")
            return False

        try:
            # Configure OpenRouter client
            openrouter_client = AsyncOpenAI(
                api_key=Config.OPENROUTER_API_KEY,
                base_url="https://openrouter.ai/api/v1"
            )

            # Configure OpenRouter model
            openrouter_model = OpenAIChatCompletionsModel(
                model=Config.MODEL_NAME,
                openai_client=openrouter_client
            )

            # Create the agent using OpenAI Agents SDK with the retrieval tool
            self.agent = Agent(
                name="BookQAAgent",
                instructions="""You are a strictly grounded RAG assistant for the 'Physical AI' book.

STRICT GROUNDING RULES:
1. MANDATORY TOOL USE: Before providing any information, you MUST call 'retrieve_book_content'.
2. NO OUTSIDE KNOWLEDGE: You must answer based EXCLUSIVELY on the text returned by the tool. Do not use your pre-training knowledge about AI, robotics, or any other topic.
3. HANDLING MISSING INFO: If the tool returns 'NO RELEVANT CONTENT FOUND', you MUST state: "I'm sorry, but I couldn't find specific information about that in the Physical AI book." Do not try to be helpful by guessing.
4. CITATION REQUIREMENT: Every factual claim must be followed by a citation in the format: (Section: [Name], Page: [Number]).
5. CONCISE ANSWERS: Keep your answers focused and directly related to the retrieved passages.

Format:
[Answer text]

Sources:
- Section: [Name], Page: [Number]""",
                model=openrouter_model,
                tools=[retrieve_book_content]
            )

            self._initialized = True
            logger.info("OpenAI Agent initialized successfully with retrieval tool")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize agent: {e}")
            self._initialized = False
            return False

    def is_initialized(self) -> bool:
        """Check if agent is initialized."""
        return self._initialized

    def generate_response_sync(self, question: str) -> str:
        """
        Generate a response synchronously using OpenAI Agents SDK Runner.run_sync().

        The agent will automatically use the retrieval tool as needed.

        Args:
            question: The user's question

        Returns:
            Generated answer from the agent
        """
        if not self._initialized or self.agent is None:
            raise AgentError("Agent not initialized")

        try:
            logger.info(f"Generating response for question: {question[:50]}...")

            # Run the agent synchronously using OpenAI Agents SDK
            result = Runner.run_sync(self.agent, question)

            return result.final_output

        except Exception as e:
            logger.error(f"Agent generation failed: {e}")
            raise AgentError(f"Generation failed: {e}")

    async def generate_response(self, question: str) -> tuple[str, bool, bool]:
        """
        Generate a response asynchronously using OpenAI Agents SDK Runner.run().

        The agent will automatically use the retrieval tool as needed.

        Args:
            question: The user's question

        Returns:
            Tuple of (answer, has_relevant_content, sources_mentioned)
        """
        if not self._initialized or self.agent is None:
            raise AgentError("Agent not initialized")

        try:
            logger.info(f"Generating response for question: {question[:50]}...")

            # Run the agent asynchronously using OpenAI Agents SDK
            result = await Runner.run(self.agent, question)

            # Check if the agent used the retrieval tool
            has_relevant_content = False
            sources_mentioned = False

            for item in result.new_items:
                if item.type == "tool_call_item":
                    has_relevant_content = True
                    sources_mentioned = True
                    break

            return (result.final_output, has_relevant_content, sources_mentioned)

        except Exception as e:
            logger.error(f"Agent generation failed: {e}")
            raise AgentError(f"Generation failed: {e}")


# ============================================================================
# Helper Functions (T014, T015, T018)
# ============================================================================

def encode_query(query: str) -> list[float]:
    """
    Convert a query string to an embedding vector using Cohere embeddings.

    Args:
        query: The query text to embed

    Returns:
        Embedding vector as list of floats (1024 dimensions)
    """
    return get_embedding(query)


def retrieve_context_chunks(query: str, qdrant_service: QdrantService, limit: int = 5) -> list[dict]:
    """
    Retrieve relevant chunks from Qdrant based on the query.

    (T014 - Retrieve context function)
    """
    query_vector = encode_query(query)
    results = qdrant_service.search(
        query_vector=query_vector,
        limit=limit,
        score_threshold=Config.SIMILARITY_THRESHOLD
    )
    return results


def format_chunks_for_context(chunks: list[dict]) -> str:
    """
    Format retrieved chunks into a context string for the agent.

    (T015 - Format chunks for context)
    """
    if not chunks:
        return "No relevant book content found."

    context_parts = []
    for i, chunk in enumerate(chunks, 1):
        metadata = chunk.get("metadata", {})
        section = metadata.get("section", "Unknown")
        page = metadata.get("page", "N/A")

        context_parts.append(
            f"[Source {i}: {section}, page {page}]\n"
            f"{chunk.get('text', '')}"
        )

    return "\n\n".join(context_parts)


def extract_sources_from_result(chunks: list[dict]) -> list[SourceCitation]:
    """
    Extract source citations from retrieved chunks.

    (T018 - Extract sources from result)
    """
    sources = []
    for chunk in chunks:
        metadata = chunk.get("metadata", {})
        text = chunk.get("text", "")

        sources.append(SourceCitation(
            chunk_id=str(chunk.get("id", "")),
            section=metadata.get("section", "Unknown"),
            page=metadata.get("page"),
            text_preview=text[:100] if text else "",
            similarity_score=chunk.get("score", 0.0)
        ))

    return sources


# ============================================================================
# Utility Functions
# ============================================================================

def create_qdrant_service() -> QdrantService:
    """Create and return a new QdrantService instance."""
    return QdrantService()


def create_rag_agent(qdrant_service: QdrantService) -> RAGAgent:
    """Create and return a new RAGAgent instance."""
    return RAGAgent(qdrant_service)
