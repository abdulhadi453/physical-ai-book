# External API Contracts

**Date**: 2025-12-26
**Feature**: 002-embedding-vector-storage

This document defines the external API contracts used by the embedding pipeline.

## 1. Cohere Embed API

### Endpoint
```
POST https://api.cohere.ai/v1/embed
```

### Authentication
```
Authorization: Bearer {COHERE_API_KEY}
```

### Request Schema
```json
{
  "model": "embed-english-v3.0",
  "texts": ["string", "..."],
  "input_type": "search_document" | "search_query",
  "truncate": "END"
}
```

**Parameters**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| model | string | Yes | Model ID: `embed-english-v3.0` |
| texts | string[] | Yes | List of texts to embed (max 96) |
| input_type | string | Yes | `search_document` for indexing, `search_query` for queries |
| truncate | string | No | Truncation strategy: `END` (default), `START`, `NONE` |

### Response Schema
```json
{
  "id": "string",
  "embeddings": [[float, ...]],
  "texts": ["string", ...],
  "meta": {
    "api_version": {"version": "string"},
    "billed_units": {"input_tokens": int}
  }
}
```

**Response Fields**:
| Field | Type | Description |
|-------|------|-------------|
| embeddings | float[][] | List of embedding vectors (1024 dimensions each) |
| texts | string[] | Echo of input texts |
| meta.billed_units.input_tokens | int | Tokens used for billing |

### Error Responses
| Status | Description | Handling |
|--------|-------------|----------|
| 400 | Invalid request | Check text format and length |
| 401 | Unauthorized | Check API key |
| 429 | Rate limit exceeded | Implement exponential backoff |
| 500 | Server error | Retry with backoff |

### Usage Example
```python
import cohere

co = cohere.Client(api_key=COHERE_API_KEY)

response = co.embed(
    model="embed-english-v3.0",
    texts=["First chunk of text", "Second chunk"],
    input_type="search_document",
    truncate="END"
)

vectors = response.embeddings  # List of 1024-dim vectors
```

---

## 2. Qdrant Cloud API

### Base URL
```
https://{cluster-id}.{region}.cloud.qdrant.io:6333
```

### Authentication
```
api-key: {QDRANT_API_KEY}
```

### 2.1 Create Collection

**Endpoint**: `PUT /collections/{collection_name}`

**Request Schema**:
```json
{
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  }
}
```

**Response**: `200 OK` on success

### 2.2 Upsert Points

**Endpoint**: `PUT /collections/{collection_name}/points`

**Request Schema**:
```json
{
  "points": [
    {
      "id": "string (UUID or hex)",
      "vector": [float, ...],
      "payload": {
        "url": "string",
        "title": "string",
        "chunk_index": int,
        "content": "string",
        "token_count": int,
        "embedded_at": "string (ISO datetime)"
      }
    }
  ]
}
```

**Parameters**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| points[].id | string | Yes | Unique point identifier |
| points[].vector | float[] | Yes | 1024-dimensional embedding |
| points[].payload | object | Yes | Metadata for the point |

**Response**: `200 OK` with operation status

### 2.3 Search Points

**Endpoint**: `POST /collections/{collection_name}/points/search`

**Request Schema**:
```json
{
  "vector": [float, ...],
  "limit": 5,
  "with_payload": true,
  "score_threshold": 0.5
}
```

**Parameters**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| vector | float[] | Yes | Query vector (1024 dimensions) |
| limit | int | No | Max results (default: 10) |
| with_payload | bool | No | Include payload in response |
| score_threshold | float | No | Minimum similarity score |

**Response Schema**:
```json
{
  "result": [
    {
      "id": "string",
      "score": float,
      "payload": {
        "url": "string",
        "title": "string",
        "content": "string",
        "chunk_index": int
      }
    }
  ],
  "status": "ok",
  "time": float
}
```

### 2.4 Get Collection Info

**Endpoint**: `GET /collections/{collection_name}`

**Response Schema**:
```json
{
  "result": {
    "status": "green",
    "vectors_count": int,
    "points_count": int,
    "config": {
      "params": {
        "vectors": {
          "size": 1024,
          "distance": "Cosine"
        }
      }
    }
  }
}
```

### Error Responses
| Status | Description | Handling |
|--------|-------------|----------|
| 400 | Invalid request | Check vector dimensions and payload format |
| 401 | Unauthorized | Check API key |
| 404 | Collection not found | Create collection first |
| 500 | Server error | Retry with backoff |

### Usage Example (Python Client)
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY
)

# Create collection
client.create_collection(
    collection_name="book_embeddings",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
)

# Upsert points
client.upsert(
    collection_name="book_embeddings",
    points=[
        PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={"url": url, "title": title, "content": text}
        )
    ]
)

# Search
results = client.search(
    collection_name="book_embeddings",
    query_vector=query_embedding,
    limit=5
)
```

---

## 3. Rate Limits and Best Practices

### Cohere
- Free tier: 100 calls/minute
- Batch up to 96 texts per call
- Use exponential backoff on 429 errors
- Cache embeddings to avoid regeneration

### Qdrant Cloud Free Tier
- 1 cluster
- 1GB storage (~100k vectors)
- No rate limits on API calls
- Use batch upsert (up to 100 points per call)

### Retry Strategy
```python
import time

def retry_with_backoff(func, max_retries=3, base_delay=1):
    for attempt in range(max_retries):
        try:
            return func()
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            delay = base_delay * (2 ** attempt)
            print(f"Retry {attempt + 1}/{max_retries} after {delay}s: {e}")
            time.sleep(delay)
```
