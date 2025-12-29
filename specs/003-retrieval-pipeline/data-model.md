# Data Model: Data Retrieval & Pipeline Validation

**Branch**: `003-retrieval-pipeline` | **Date**: 2025-12-27

## Entity Overview

```
┌─────────────────┐      ┌───────────────────┐      ┌──────────────────┐
│  SearchQuery    │─────▶│   QueryEmbedding  │─────▶│   SearchResult   │
│  (input)        │      │   (Cohere)        │      │   (output)       │
└─────────────────┘      └───────────────────┘      └──────────────────┘
                                   │
                                   ▼
                         ┌───────────────────┐
                         │  Qdrant Vector DB │
                         │  (Physical-AI-Book)│
                         └───────────────────┘
```

## Core Entities

### SearchQuery

The input query from a developer for semantic search.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| text | str | 1-8000 chars, non-empty | Natural language query |
| top_k | int | 1-50, default 5 | Number of results to return |
| score_threshold | float | 0.0-1.0, default 0.7 | Minimum similarity score |

**Validation Rules**:
- Query text must be non-empty after stripping whitespace
- Query text must not exceed 8000 characters (Cohere limit)
- top_k must be positive integer, capped at 50 for performance
- score_threshold must be between 0.0 and 1.0 inclusive

### SearchResult

A single result from the vector similarity search.

| Field | Type | Description |
|-------|------|-------------|
| chunk_id | str | Unique identifier (MD5 hash) |
| content | str | Text content of the chunk |
| source_url | str | URL of the source page |
| source_title | str | Title of the source page |
| chunk_index | int | Position within source page |
| token_count | int | Estimated token count |
| similarity_score | float | Cosine similarity (0.0-1.0) |
| embedded_at | str | ISO timestamp of embedding |

**Relationships**:
- References existing Chunk entity from embedding pipeline
- Augmented with similarity_score from Qdrant search

### RetrievalConfig

Configuration for the retrieval pipeline (singleton).

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| qdrant_url | str | env | Qdrant Cloud endpoint URL |
| qdrant_api_key | str | env | Qdrant authentication key |
| collection_name | str | "Physical-AI-Book" | Target collection |
| cohere_api_key | str | env | Cohere API key |
| embedding_model | str | "embed-english-v3.0" | Cohere model name |
| vector_size | int | 1024 | Expected vector dimensions |
| default_top_k | int | 5 | Default results limit |
| default_threshold | float | 0.7 | Default score threshold |
| max_retries | int | 3 | Retry attempts for API calls |
| retry_delay | float | 1.0 | Base delay between retries |

### RetrievalResponse

Wrapper for search operation results.

| Field | Type | Description |
|-------|------|-------------|
| query | str | Original query text |
| results | list[SearchResult] | List of matching chunks |
| total_found | int | Number of results returned |
| search_time_ms | float | Search duration in milliseconds |
| status | str | "success" or "no_matches" |
| message | str | Human-readable status message |

**Status Values**:
- `"success"`: Results found above threshold
- `"no_matches"`: Search succeeded but no results above threshold
- `"error"`: Search failed (connection, API error)

### ValidationResult

Result from test harness validation.

| Field | Type | Description |
|-------|------|-------------|
| query | str | Test query text |
| expected_keywords | list[str] | Keywords expected in results |
| passed | bool | Whether validation passed |
| results_count | int | Number of results returned |
| top_result_score | float | Similarity score of best match |
| keywords_found | list[str] | Expected keywords found in results |
| execution_time_ms | float | Query execution time |

## State Transitions

```
Query Received
      │
      ▼
┌─────────────────┐
│   VALIDATING    │──────▶ ValidationError (invalid input)
└─────────────────┘
      │ valid
      ▼
┌─────────────────┐
│   EMBEDDING     │──────▶ EmbeddingError (Cohere failure)
└─────────────────┘
      │ success
      ▼
┌─────────────────┐
│   SEARCHING     │──────▶ SearchError (Qdrant failure)
└─────────────────┘
      │ success
      ▼
┌─────────────────┐
│   FORMATTING    │
└─────────────────┘
      │
      ▼
┌─────────────────┐
│   COMPLETE      │──────▶ RetrievalResponse
└─────────────────┘
```

## Existing Qdrant Payload Schema

Reference: Chunks stored by embedding pipeline (Spec 1)

```python
payload = {
    "url": str,           # Source page URL
    "title": str,         # Page title
    "chunk_index": int,   # Position in page (0-based)
    "content": str,       # Chunk text content
    "token_count": int,   # Estimated tokens
    "embedded_at": str    # ISO timestamp
}
```

**Vector Properties**:
- Dimensions: 1024 (Cohere embed-english-v3.0)
- Distance: Cosine similarity
- ID format: MD5 hash of `{url}_{chunk_index}`
