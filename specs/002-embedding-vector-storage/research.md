# Research: Book Content Embedding and Vector Storage

**Date**: 2025-12-26
**Feature**: 002-embedding-vector-storage
**Status**: Complete

## Research Questions Addressed

### 1. Project Setup with UV

**Decision**: Use `uv` as the Python package manager and project initializer

**Rationale**:
- `uv` is an extremely fast Python package manager written in Rust
- Provides `uv init` for project initialization with standard pyproject.toml
- Handles virtual environment creation automatically
- Compatible with pip-style requirements and modern Python packaging standards
- Much faster than pip/poetry for dependency resolution and installation

**Alternatives Considered**:
- **pip + venv**: Traditional but slower, less modern pyproject.toml support
- **poetry**: Good but slower than uv, heavier dependency
- **conda**: Overkill for this use case, not needed for pure Python packages

**Implementation**:
```bash
cd backend/
uv init
uv add cohere qdrant-client beautifulsoup4 requests python-dotenv
```

### 2. Web Scraping Strategy for Docusaurus

**Decision**: Use `requests` + `BeautifulSoup4` for HTML fetching and parsing

**Rationale**:
- Docusaurus generates static HTML pages, no JavaScript rendering needed
- BeautifulSoup4 is lightweight and well-suited for HTML parsing
- `requests` handles HTTP efficiently with built-in retry support
- No need for heavy browser automation (Selenium, Playwright)

**Alternatives Considered**:
- **Scrapy**: Overkill for a single-site crawl, adds complexity
- **Selenium/Playwright**: Unnecessary since Docusaurus serves static HTML
- **httpx**: Modern alternative to requests, but requests is simpler and sufficient

**Content Extraction Strategy**:
- Target `<article>` or `<main>` tags where Docusaurus places content
- Remove navigation (`<nav>`), footer (`<footer>`), and sidebar elements
- Extract page title from `<h1>` or `<title>` tag
- Preserve heading structure for semantic chunking

### 3. Text Chunking Strategy

**Decision**: Fixed-size chunking with token count (500 tokens, 100 token overlap)

**Rationale**:
- 500 tokens is optimal for Cohere embed-english-v3.0 (max 512 tokens)
- 100 token overlap (20%) preserves context across chunk boundaries
- Simple to implement and debug
- Consistent chunk sizes improve embedding quality
- Good balance between context preservation and search granularity

**Alternatives Considered**:
- **Semantic chunking**: More complex, requires NLP models, overkill for this scope
- **Sentence-based**: Variable sizes lead to inconsistent embeddings
- **Paragraph-based**: May create chunks too large or too small

**Implementation**:
- Use character-based splitting with estimated token count (4 chars ≈ 1 token)
- Split on sentence boundaries when possible to preserve semantic units
- Target ~2000 characters per chunk (≈500 tokens)
- Overlap ~400 characters (≈100 tokens)

### 4. Cohere Embedding Model Selection

**Decision**: Use `embed-english-v3.0` model with `input_type="search_document"`

**Rationale**:
- `embed-english-v3.0` is Cohere's latest English embedding model
- Produces 1024-dimensional vectors with excellent semantic quality
- Supports batch embedding (up to 96 texts per API call) for efficiency
- `input_type` parameter optimizes embeddings for search vs retrieval
- Free tier allows sufficient API calls for typical book size

**Model Specs**:
- Dimensions: 1024
- Max tokens: 512 per input
- Batch size: Up to 96 texts
- Input types: `search_document` (for indexing), `search_query` (for queries)

**Alternatives Considered**:
- **embed-multilingual-v3.0**: Not needed for English-only content
- **OpenAI ada-002**: Requires separate API key, different ecosystem
- **Local models (sentence-transformers)**: Adds complexity, slower

### 5. Qdrant Cloud Configuration

**Decision**: Use Qdrant Cloud free tier with Cosine distance metric

**Rationale**:
- Free tier provides 1GB storage, sufficient for ~100k vectors
- Cloud-hosted eliminates infrastructure management
- Cosine similarity is standard for semantic search
- Native Python client with simple API
- Supports metadata filtering for source attribution

**Collection Configuration**:
- Vector size: 1024 (matches Cohere embed-english-v3.0)
- Distance metric: Cosine
- Payload indexing: Enable on `url`, `title` for filtering

**Alternatives Considered**:
- **Self-hosted Qdrant**: Requires Docker, more setup
- **Pinecone**: Requires different API, similar free tier limits
- **ChromaDB**: Local-only, doesn't persist well for production

### 6. Single File Architecture

**Decision**: Implement all functionality in a single `main.py` with clear function organization

**Rationale**:
- User explicitly requested single `main.py` file
- Reduces complexity for hackathon scope
- Easier to understand and debug
- All dependencies in one place
- Suitable for script-based pipeline execution

**File Organization**:
```python
# main.py structure
# 1. Configuration and imports
# 2. URL Discovery and Crawling functions
# 3. Content Extraction and Cleaning functions
# 4. Text Chunking functions
# 5. Embedding Generation functions
# 6. Qdrant Storage functions
# 7. Validation/Test Query functions
# 8. Main orchestration function
```

### 7. Error Handling and Retry Strategy

**Decision**: Implement exponential backoff retry (3 attempts, 1s/2s/4s delays)

**Rationale**:
- Handles transient API failures gracefully
- Prevents overwhelming external services
- Standard pattern for API integrations
- Easy to implement with simple loop
- Logs failures clearly for debugging

**Implementation**:
- Wrap API calls (Cohere, Qdrant) in retry decorator/function
- Log each retry attempt with error details
- Fail gracefully after max retries with clear error message

### 8. Idempotency Strategy

**Decision**: Use URL-based deduplication with Qdrant upsert

**Rationale**:
- Each chunk gets a deterministic ID based on URL + chunk_index
- Qdrant `upsert` operation handles duplicates automatically
- Re-running pipeline updates existing vectors rather than duplicating
- Simple to implement without external state management

**ID Generation**:
```python
chunk_id = hashlib.md5(f"{url}_{chunk_index}".encode()).hexdigest()
```

### 9. Environment Configuration

**Decision**: Use `.env` file with `python-dotenv` for configuration

**Rationale**:
- Standard pattern for API key management
- Keeps secrets out of code
- Easy to configure across environments
- Simple library with no dependencies

**Required Environment Variables**:
- `COHERE_API_KEY`: Cohere API authentication
- `QDRANT_URL`: Qdrant cloud cluster URL
- `QDRANT_API_KEY`: Qdrant authentication
- `BOOK_BASE_URL`: Docusaurus site root URL (e.g., https://book.vercel.app)
- `QDRANT_COLLECTION_NAME`: Collection name for vectors (default: "book_embeddings")

## Constitution Compliance Check

| Constitution Requirement | Compliance Status |
|--------------------------|-------------------|
| Spec-Driven Development | ✅ Implementation based on approved spec |
| RAG stack: Qdrant required | ✅ Using Qdrant Cloud as specified |
| Content chunked for RAG | ✅ 500-token chunks with overlap |
| Technical rigor | ✅ Clear separation of concerns, error handling |
| Reproducibility | ✅ Deterministic chunk IDs, idempotent operations |
| Safety | ✅ No destructive operations, read-only crawling |

## Summary

All research questions resolved. The technical stack is:
- **Project**: `uv` for Python package management, `backend/` folder structure
- **Scraping**: `requests` + `BeautifulSoup4` for Docusaurus HTML
- **Chunking**: Fixed 500-token chunks with 100-token overlap
- **Embeddings**: Cohere `embed-english-v3.0` (1024 dimensions)
- **Storage**: Qdrant Cloud free tier with Cosine similarity
- **Architecture**: Single `main.py` with orchestration function
- **Config**: `.env` file with `python-dotenv`
