# Implementation Plan: Data Retrieval & Pipeline Validation

**Branch**: `003-retrieval-pipeline` | **Date**: 2025-12-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-retrieval-pipeline/spec.md`

## Summary

Implement a single `retrieve.py` file that enables semantic search against the existing Qdrant collection (populated by Spec 1). The file will reuse the existing backend `.env` configuration for Cohere and Qdrant credentials, generate query embeddings, perform vector similarity search, and return ranked results with metadata. Includes a built-in test harness for pipeline validation.

## Technical Context

**Language/Version**: Python 3.13 (matching existing backend)
**Primary Dependencies**: cohere>=5.20.1, qdrant-client>=1.16.2, python-dotenv>=1.2.1
**Storage**: Qdrant Cloud (existing `Physical-AI-Book` collection)
**Testing**: Built-in validation harness with test queries
**Target Platform**: CLI script (Linux/Windows/macOS)
**Project Type**: Single file addition to existing backend
**Performance Goals**: <2 seconds per query (95th percentile)
**Constraints**: Reuse existing .env configuration, no new dependencies
**Scale/Scope**: Single file (~200-300 lines), 5 test queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| Spec-Driven Development | ✅ PASS | Implementing from approved spec.md |
| AI-Native Integration | ✅ PASS | RAG retrieval component for learning system |
| Technical Rigor | ✅ PASS | Uses established embedding model, validated search |
| Content Chunked for RAG | ✅ PASS | Retrieves existing chunks for RAG pipeline |
| Verification & Reproducibility | ✅ PASS | Includes validation test harness |

## Project Structure

### Documentation (this feature)

```text
specs/003-retrieval-pipeline/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions
├── quickstart.md        # Usage guide
└── checklists/
    └── requirements.md  # Quality checklist
```

### Source Code (repository root)

```text
backend/
├── retrieve.py          # NEW - Retrieval pipeline (this implementation)
├── main.py              # Existing - Embedding pipeline (Spec 1)
├── .env                 # Existing - API keys and configuration
├── pyproject.toml       # Existing - Dependencies
└── README.md            # Existing - Documentation
```

**Structure Decision**: Single file addition to existing `backend/` directory. Reuses existing `.env` configuration and dependencies from `pyproject.toml`. No new directories or configuration files required.

## Implementation Design

### File: `retrieve.py`

**Purpose**: Standalone retrieval script with CLI interface and programmatic API

**Sections**:

1. **Imports & Configuration** (~30 lines)
   - Load environment variables from `.env`
   - Initialize CONFIG dict (same pattern as main.py)
   - Validate required credentials

2. **Data Classes** (~20 lines)
   - `SearchResult` dataclass for structured results
   - `RetrievalResponse` for API responses

3. **Utilities** (~30 lines)
   - `log()` - Timestamped logging
   - `retry_with_backoff()` - Retry decorator for API calls
   - `validate_query()` - Input validation

4. **Core Functions** (~80 lines)
   - `embed_query(query)` - Generate embedding via Cohere
   - `search_vectors(vector, limit, threshold)` - Search Qdrant
   - `retrieve(query, top_k, threshold)` - Main retrieval function
   - `format_results(results)` - Format for display

5. **Validation Harness** (~50 lines)
   - `TEST_QUERIES` - List of validation queries with expected keywords
   - `validate_pipeline()` - Run all test queries
   - `check_keywords()` - Verify expected content in results

6. **CLI Interface** (~40 lines)
   - Argument parsing (query, --top-k, --threshold, --validate)
   - Main entry point
   - Output formatting

### Configuration (from existing .env)

```python
CONFIG = {
    "cohere_api_key": os.getenv("COHERE_API_KEY"),
    "qdrant_url": os.getenv("QDRANT_URL"),
    "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
    "collection_name": os.getenv("QDRANT_COLLECTION_NAME", "Physical-AI-Book"),
    "embedding_model": "embed-english-v3.0",
    "vector_size": 1024,
    "default_top_k": 5,
    "default_threshold": 0.7,
    "max_retries": 3,
    "retry_delay": 1.0,
}
```

### Test Queries for Validation

| Query | Expected Keywords | Purpose |
|-------|-------------------|---------|
| "What is Physical AI?" | physical, AI, robot | Core concept |
| "How do robots learn from demonstrations?" | learn, demonstration, imitation | VLA/learning |
| "Robot safety considerations" | safety, risk, constraint | Ethics |
| "ROS 2 architecture" | ROS, node, topic | Framework |
| "Sim-to-real transfer" | simulation, real, transfer | Deployment |

## Error Handling Strategy

| Error Type | Detection | Response |
|------------|-----------|----------|
| Empty query | `not query.strip()` | ValueError with message |
| Query too long | `len(query) > 8000` | ValueError with message |
| Cohere unavailable | API exception | Retry 3x, then ConnectionError |
| Qdrant unavailable | Client exception | Retry 3x, then ConnectionError |
| No results | Empty search results | Return success with empty list |
| Below threshold | All scores < threshold | Return success with empty list |

## Complexity Tracking

No constitution violations - single file implementation with minimal complexity.

## Dependencies

All dependencies already exist in `pyproject.toml`:

| Package | Version | Used For |
|---------|---------|----------|
| cohere | >=5.20.1 | Query embedding generation |
| qdrant-client | >=1.16.2 | Vector similarity search |
| python-dotenv | >=1.2.1 | Load .env configuration |

## Acceptance Criteria

- [ ] `retrieve.py` executes without errors
- [ ] Queries return results within 2 seconds
- [ ] Results include content, source, score, and metadata
- [ ] Empty/invalid queries return clear error messages
- [ ] `--validate` flag runs test suite successfully
- [ ] All 5 test queries return relevant results (>0.7 score)
- [ ] Reuses existing `.env` configuration (no new config files)
