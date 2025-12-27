# Implementation Plan: Book Content Embedding and Vector Storage

**Branch**: `002-embedding-vector-storage` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-embedding-vector-storage/spec.md`

## Summary

Implement a Python-based pipeline to crawl a deployed Docusaurus book, extract and clean text content, chunk it into semantic units, generate vector embeddings using Cohere, and store them in Qdrant Cloud for semantic search. The implementation uses a single `main.py` file in a `backend/` folder, initialized with `uv` package manager.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client, beautifulsoup4, requests, python-dotenv
**Storage**: Qdrant Cloud (free tier) - no local database
**Testing**: Manual validation with test queries (no pytest framework for this scope)
**Target Platform**: CLI script (cross-platform: Linux, macOS, Windows)
**Project Type**: single (backend only)
**Performance Goals**: Complete pipeline in <60 minutes for 100-200 pages; <500ms per search query
**Constraints**: Cohere free tier rate limits (100 calls/min), Qdrant free tier (1GB storage)
**Scale/Scope**: 100-200 pages, 1000-5000 vectors, single collection

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Requirement | Status | Notes |
|-------------|--------|-------|
| Spec-Driven Development | ✅ PASS | Implementation based on approved spec |
| RAG stack: Qdrant required | ✅ PASS | Using Qdrant Cloud as mandated |
| Content chunked for RAG | ✅ PASS | 500-token chunks with overlap |
| Technical rigor | ✅ PASS | Clear error handling, retry logic |
| Reproducibility | ✅ PASS | Idempotent operations, deterministic IDs |
| Safety | ✅ PASS | Read-only crawling, no destructive ops |
| Claude Code and Spec-Kit Plus | ✅ PASS | Using defined workflow |

**Gate Status**: ✅ PASSED - Proceeding with implementation planning

## Project Structure

### Documentation (this feature)

```text
specs/002-embedding-vector-storage/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output - technical decisions
├── data-model.md        # Phase 1 output - data structures
├── quickstart.md        # Phase 1 output - setup guide
├── contracts/           # Phase 1 output
│   └── external-apis.md # Cohere and Qdrant API contracts
├── checklists/
│   └── requirements.md  # Spec validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── pyproject.toml       # uv project configuration
├── .env                 # Environment variables (gitignored)
├── .env.example         # Environment template
├── main.py              # Single file with full pipeline
└── .python-version      # Python version pin
```

**Structure Decision**: Single-file architecture per user request. All functionality in `main.py` with clear function organization. No separate modules, tests directory, or package structure.

## Implementation Architecture

### Main.py Function Organization

```python
# main.py structure (~400-500 lines)

# ============================================================
# Section 1: Imports and Configuration (~30 lines)
# ============================================================
# - Standard library imports
# - Third-party imports (cohere, qdrant_client, bs4, requests)
# - Environment variable loading with dotenv
# - Configuration dataclass/dict

# ============================================================
# Section 2: Data Classes (~40 lines)
# ============================================================
# - BookPage dataclass
# - TextChunk dataclass
# - Configuration validation

# ============================================================
# Section 3: URL Discovery and Crawling (~80 lines)
# ============================================================
# - discover_urls(base_url) -> List[str]
# - fetch_page(url) -> Optional[str]
# - crawl_site(base_url) -> List[BookPage]

# ============================================================
# Section 4: Content Extraction (~60 lines)
# ============================================================
# - extract_content(html) -> Tuple[str, str]  # title, content
# - clean_html(soup) -> str

# ============================================================
# Section 5: Text Chunking (~50 lines)
# ============================================================
# - chunk_text(content, url, title) -> List[TextChunk]
# - estimate_tokens(text) -> int

# ============================================================
# Section 6: Embedding Generation (~60 lines)
# ============================================================
# - generate_embeddings(chunks, cohere_client) -> List[Tuple[TextChunk, List[float]]]
# - batch_embed(texts, client) -> List[List[float]]

# ============================================================
# Section 7: Qdrant Storage (~70 lines)
# ============================================================
# - setup_collection(client, collection_name)
# - store_embeddings(client, collection_name, chunk_vectors)
# - upsert_batch(client, collection_name, points)

# ============================================================
# Section 8: Validation (~40 lines)
# ============================================================
# - validate_search(client, cohere_client, collection_name)
# - run_test_queries(client, cohere_client, queries)

# ============================================================
# Section 9: Main Orchestration (~50 lines)
# ============================================================
# - main() -> orchestrates full pipeline
# - if __name__ == "__main__": main()
```

### Key Implementation Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Package manager | uv | Fast, modern, simple init |
| HTML parser | BeautifulSoup4 | Lightweight, Docusaurus-compatible |
| Chunking strategy | Fixed 500 tokens, 100 overlap | Optimal for Cohere model |
| Embedding model | embed-english-v3.0 | Latest Cohere model, 1024 dims |
| ID generation | MD5(url + chunk_index) | Deterministic, idempotent |
| Distance metric | Cosine | Standard for semantic search |
| Error handling | 3 retries, exponential backoff | Handles transient API failures |

### Data Flow

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           main() Orchestration                           │
└──────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ Step 1: crawl_site(BOOK_BASE_URL)                                        │
│   └── discover_urls() → fetch_page() → extract_content()                 │
│   └── Output: List[BookPage]                                             │
└──────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ Step 2: chunk_text() for each BookPage                                   │
│   └── Split content into 500-token chunks with 100-token overlap         │
│   └── Output: List[TextChunk]                                            │
└──────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ Step 3: generate_embeddings(chunks)                                      │
│   └── Batch texts (96 per API call) → Cohere embed API                   │
│   └── Output: List[Tuple[TextChunk, List[float]]]                        │
└──────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ Step 4: setup_collection() + store_embeddings()                          │
│   └── Create/recreate Qdrant collection → Upsert vectors with payload    │
│   └── Output: Vectors stored in Qdrant Cloud                             │
└──────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ Step 5: validate_search()                                                │
│   └── Run test queries → Verify relevant results returned                │
│   └── Output: Validation report                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

## Complexity Tracking

No complexity violations. Implementation follows constitution guidelines:
- Single project (not multi-project)
- No unnecessary abstractions
- Direct API calls without repository pattern
- Minimal dependencies

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Cohere rate limits | Batch 96 texts/call, exponential backoff |
| Qdrant free tier limits | Monitor vector count, efficient chunking |
| Network failures | 3 retries with increasing delays |
| Stale content | Idempotent upsert, re-runnable pipeline |

## Dependencies Map

```
backend/main.py
    │
    ├── cohere (Python SDK)
    │   └── api.cohere.ai (external)
    │
    ├── qdrant-client (Python SDK)
    │   └── Qdrant Cloud (external)
    │
    ├── beautifulsoup4
    │   └── lxml (parser backend)
    │
    ├── requests
    │   └── Docusaurus site (external)
    │
    └── python-dotenv
        └── .env file (local)
```

## Files to Create

| File | Purpose | Lines (est.) |
|------|---------|--------------|
| `backend/pyproject.toml` | uv project config | ~15 |
| `backend/.env.example` | Environment template | ~10 |
| `backend/main.py` | Full pipeline implementation | ~450 |

## Next Steps

Run `/sp.tasks` to generate the implementation task list with test cases.

## Related Artifacts

- [Specification](./spec.md) - Feature requirements
- [Research](./research.md) - Technical decisions and rationale
- [Data Model](./data-model.md) - Data structures
- [API Contracts](./contracts/external-apis.md) - External API specs
- [Quickstart](./quickstart.md) - Setup and usage guide
