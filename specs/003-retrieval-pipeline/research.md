# Research: Data Retrieval & Pipeline Validation

**Branch**: `003-retrieval-pipeline` | **Date**: 2025-12-27

## Research Tasks Completed

### 1. Existing Backend Structure Analysis

**Decision**: Reuse existing `main.py` patterns and create standalone `retrieve.py`

**Rationale**:
- `main.py` (19KB) contains a complete embedding pipeline with query functions already available
- Functions `embed_query()` and `search_vectors()` exist and are working
- Same configuration pattern can be reused (.env, CONFIG dict)
- Creating a separate file maintains single-responsibility principle

**Alternatives Considered**:
- Import from main.py: Rejected because main.py is an ingestion script, not a library
- Extend main.py: Rejected because it adds complexity to ingestion workflow
- Create package structure: Over-engineering for single retrieval file

### 2. Qdrant Collection Configuration

**Decision**: Connect to existing `Physical-AI-Book` collection

**Rationale**:
- Collection already exists from Spec 1 implementation
- Vector size: 1024 dimensions (Cohere embed-english-v3.0)
- Distance metric: Cosine similarity
- Contains embedded chunks with metadata (url, title, chunk_index, content, token_count, embedded_at)

**Configuration Found**:
```python
QDRANT_URL = "https://60d01170-c5c2-4513-a7a6-..."  # Cloud endpoint
QDRANT_API_KEY = "..."  # JWT token
QDRANT_COLLECTION_NAME = "Physical-AI-Book"
```

### 3. Cohere Embedding Model

**Decision**: Use `embed-english-v3.0` (same as ingestion)

**Rationale**:
- Ensures query embeddings match stored vector dimensions (1024)
- Model consistency is critical for similarity search accuracy
- Already configured in existing codebase

**Model Configuration**:
```python
COHERE_API_KEY = "..."
EMBEDDING_MODEL = "embed-english-v3.0"
INPUT_TYPE = "search_query"  # Different from "search_document" used in ingestion
```

### 4. Search Parameters Best Practices

**Decision**: Default top-k=5, configurable threshold=0.7

**Rationale**:
- Top-5 results balance relevance with information density
- 0.7 cosine threshold filters noise while keeping useful results
- Both configurable for tuning during validation

**Qdrant Search Configuration**:
```python
search_params = SearchParams(
    hnsw_ef=128,  # Search recall quality
    exact=False   # Approximate search for speed
)
limit = 5  # Default top-k
score_threshold = 0.7  # Minimum relevance
```

### 5. Error Handling Strategy

**Decision**: Three-tier error handling with clear user messages

**Rationale**:
- Tier 1: Validation errors (empty query, too long) → 400-level message
- Tier 2: Connection errors (Qdrant, Cohere unavailable) → Retry + descriptive error
- Tier 3: No results (below threshold) → Success with empty list + explanation

**Error Categories**:
| Error Type | User Message | Action |
|------------|--------------|--------|
| Empty query | "Query cannot be empty" | Return immediately |
| Query too long | "Query exceeds 8000 character limit" | Return immediately |
| Cohere unavailable | "Embedding service unavailable, try again" | Retry 3x, then fail |
| Qdrant unavailable | "Vector database connection failed" | Retry 3x, then fail |
| No matches | "No relevant results found above threshold" | Return empty list |

### 6. Testing Strategy

**Decision**: Built-in test harness with 5 validation queries

**Rationale**:
- Test queries cover different aspects of Physical AI content
- Enables immediate validation after implementation
- Matches FR-010 requirement for programmatic test harness

**Test Queries**:
1. "What is Physical AI?" - Core concept, should return definitional content
2. "How do robots learn from human demonstrations?" - VLA/imitation learning
3. "What are the safety considerations for humanoid robots?" - Ethics/safety content
4. "Explain the ROS 2 architecture" - Technical framework content
5. "What is sim-to-real transfer?" - Simulation/deployment concept

## Dependencies Confirmed

| Dependency | Version | Purpose |
|------------|---------|---------|
| cohere | >=5.20.1 | Query embedding generation |
| qdrant-client | >=1.16.2 | Vector search operations |
| python-dotenv | >=1.2.1 | Environment configuration |

## No Unresolved Clarifications

All technical decisions have been made based on existing codebase analysis.
