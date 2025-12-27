# Data Model: Book Content Embedding and Vector Storage

**Date**: 2025-12-26
**Feature**: 002-embedding-vector-storage
**Status**: Final

## Overview

This document defines the data structures used throughout the embedding pipeline. Since this is a script-based pipeline (not a web service), data is represented as Python dataclasses and Qdrant payloads rather than database schemas.

## Core Entities

### 1. BookPage

Represents a single page crawled from the Docusaurus site.

```python
@dataclass
class BookPage:
    """A page extracted from the Docusaurus book."""
    url: str                    # Unique identifier, full URL of the page
    title: str                  # Page title from <h1> or <title> tag
    content: str                # Cleaned text content (HTML removed)
    crawled_at: datetime        # Timestamp of when page was crawled
```

**Validation Rules**:
- `url` must be a valid HTTP/HTTPS URL within the book domain
- `title` must be non-empty (fallback to URL slug if missing)
- `content` must be non-empty after cleaning

**Relationships**:
- One BookPage produces many TextChunks

### 2. TextChunk

Represents a semantic unit of text ready for embedding.

```python
@dataclass
class TextChunk:
    """A chunk of text extracted from a BookPage."""
    chunk_id: str               # Deterministic ID: md5(url + chunk_index)
    source_url: str             # URL of the source BookPage
    source_title: str           # Title of the source page
    chunk_index: int            # Position within the page (0-indexed)
    content: str                # Text content of the chunk (200-1000 tokens)
    token_count: int            # Estimated token count (chars / 4)
```

**Validation Rules**:
- `chunk_id` must be unique across all chunks
- `content` length should be 800-4000 characters (200-1000 tokens)
- `chunk_index` starts at 0 and increments sequentially per page

**ID Generation**:
```python
import hashlib
chunk_id = hashlib.md5(f"{source_url}_{chunk_index}".encode()).hexdigest()
```

### 3. VectorPoint

Represents the data stored in Qdrant for a single embedding.

```python
@dataclass
class VectorPoint:
    """A point to be stored in Qdrant."""
    id: str                     # Same as TextChunk.chunk_id
    vector: List[float]         # 1024-dimensional embedding from Cohere
    payload: Dict[str, Any]     # Metadata stored with the vector
```

**Payload Structure**:
```python
payload = {
    "url": str,                 # Source URL for attribution
    "title": str,               # Page title for display
    "chunk_index": int,         # Position in source page
    "content": str,             # Original text (for retrieval display)
    "token_count": int,         # Token estimate for reference
    "embedded_at": str          # ISO timestamp of embedding creation
}
```

**Validation Rules**:
- `vector` must have exactly 1024 dimensions
- `payload["url"]` must match the chunk's source URL
- `payload["content"]` must be non-empty

### 4. SearchResult

Represents a result returned from a Qdrant vector search.

```python
@dataclass
class SearchResult:
    """A result from vector search."""
    chunk_id: str               # ID of the matching chunk
    score: float                # Similarity score (0.0 to 1.0 for cosine)
    url: str                    # Source URL from payload
    title: str                  # Page title from payload
    content: str                # Text content from payload
    chunk_index: int            # Position in source page
```

**Validation Rules**:
- `score` should be between 0.0 and 1.0 for cosine similarity
- Higher scores indicate greater semantic similarity

## Qdrant Collection Schema

### Collection: `book_embeddings`

**Configuration**:
```python
collection_config = {
    "name": "book_embeddings",
    "vectors": {
        "size": 1024,           # Cohere embed-english-v3.0 dimensions
        "distance": "Cosine"    # Similarity metric
    }
}
```

**Payload Indexes** (for filtering):
- `url` (keyword): Filter by source page
- `title` (keyword): Filter by page title

## Data Flow

```
┌─────────────┐     ┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│  Docusaurus │────▶│  BookPage   │────▶│  TextChunk   │────▶│ VectorPoint │
│    Site     │     │  (crawled)  │     │  (chunked)   │     │  (embedded) │
└─────────────┘     └─────────────┘     └──────────────┘     └──────┬──────┘
                                                                    │
                                                                    ▼
                                                             ┌─────────────┐
                                                             │   Qdrant    │
                                                             │   Cloud     │
                                                             └─────────────┘
```

## State Transitions

### Page Lifecycle
1. **Discovered**: URL found via link traversal
2. **Fetched**: HTML content retrieved
3. **Extracted**: Clean text extracted from HTML
4. **Chunked**: Text split into semantic units
5. **Embedded**: Vectors generated for chunks
6. **Stored**: Vectors uploaded to Qdrant

### Error States
- **Fetch Failed**: HTTP error or timeout during crawl
- **Empty Content**: Page had no extractable text
- **Embedding Failed**: Cohere API error
- **Storage Failed**: Qdrant upload error

## Example Data

### BookPage Example
```python
BookPage(
    url="https://physical-ai-book.vercel.app/docs/chapter1/introduction",
    title="Introduction to Physical AI",
    content="Physical AI represents a paradigm shift in how we approach...",
    crawled_at=datetime(2025, 12, 26, 10, 30, 0)
)
```

### TextChunk Example
```python
TextChunk(
    chunk_id="a1b2c3d4e5f6...",
    source_url="https://physical-ai-book.vercel.app/docs/chapter1/introduction",
    source_title="Introduction to Physical AI",
    chunk_index=0,
    content="Physical AI represents a paradigm shift in how we approach artificial intelligence. Unlike traditional AI systems that operate purely in the digital realm...",
    token_count=487
)
```

### VectorPoint Payload Example
```python
{
    "url": "https://physical-ai-book.vercel.app/docs/chapter1/introduction",
    "title": "Introduction to Physical AI",
    "chunk_index": 0,
    "content": "Physical AI represents a paradigm shift...",
    "token_count": 487,
    "embedded_at": "2025-12-26T10:35:00Z"
}
```

## Notes

- No persistent local database is used; all state is either in-memory or in Qdrant
- Re-running the pipeline uses Qdrant upsert for idempotency
- Crawled pages are processed in streaming fashion to minimize memory usage
