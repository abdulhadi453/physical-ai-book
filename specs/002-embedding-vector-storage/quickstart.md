# Quickstart: Book Content Embedding Pipeline

**Date**: 2025-12-26
**Feature**: 002-embedding-vector-storage

## Prerequisites

1. **Python 3.8+** installed
2. **uv** package manager installed ([install guide](https://github.com/astral-sh/uv))
3. **Cohere API Key** from [cohere.com](https://dashboard.cohere.com/api-keys)
4. **Qdrant Cloud Account** from [cloud.qdrant.io](https://cloud.qdrant.io/)
5. **Deployed Docusaurus book** on Vercel (or accessible URL)

## Setup Steps

### 1. Create Project Structure

```bash
# From repository root
mkdir backend
cd backend

# Initialize Python project with uv
uv init
```

### 2. Install Dependencies

```bash
uv add cohere qdrant-client beautifulsoup4 requests python-dotenv
```

### 3. Create Environment File

Create `backend/.env`:

```env
# Cohere API
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud
QDRANT_URL=https://your-cluster-id.region.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_embeddings

# Book URL
BOOK_BASE_URL=https://your-book.vercel.app
```

### 4. Obtain API Keys

#### Cohere API Key
1. Go to [Cohere Dashboard](https://dashboard.cohere.com/api-keys)
2. Sign up or log in
3. Generate a new API key
4. Copy to `COHERE_API_KEY` in `.env`

#### Qdrant Cloud
1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a free cluster
3. Copy the cluster URL to `QDRANT_URL`
4. Generate an API key and copy to `QDRANT_API_KEY`

### 5. Run the Pipeline

```bash
cd backend
uv run python main.py
```

## Expected Output

```
Starting embedding pipeline...
[1/4] Crawling book at https://your-book.vercel.app...
  Found 45 pages
  Extracted content from 45 pages
[2/4] Chunking content...
  Created 287 chunks
[3/4] Generating embeddings...
  Embedded batch 1/3 (96 chunks)
  Embedded batch 2/3 (96 chunks)
  Embedded batch 3/3 (95 chunks)
[4/4] Storing in Qdrant...
  Upserted 287 vectors to collection 'book_embeddings'

Pipeline complete!
Total pages: 45
Total chunks: 287
Collection: book_embeddings

Running validation queries...
Query: "What is physical AI?"
  Result 1 (0.87): Introduction to Physical AI - "Physical AI represents..."
  Result 2 (0.82): Core Concepts - "The fundamental principles of..."

Validation: PASSED (3/3 queries returned relevant results)
```

## Verification

### Check Qdrant Collection

```python
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

info = client.get_collection("book_embeddings")
print(f"Vectors stored: {info.points_count}")
```

### Run a Test Search

```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

# Generate query embedding
query = "How do robots perceive their environment?"
query_embedding = co.embed(
    model="embed-english-v3.0",
    texts=[query],
    input_type="search_query"
).embeddings[0]

# Search Qdrant
results = client.search(
    collection_name="book_embeddings",
    query_vector=query_embedding,
    limit=5
)

for r in results:
    print(f"Score: {r.score:.2f} | {r.payload['title']}")
    print(f"  {r.payload['content'][:100]}...")
```

## Troubleshooting

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| `401 Unauthorized` | Invalid API key | Check `.env` keys are correct |
| `429 Rate Limited` | Too many Cohere calls | Wait and retry, reduce batch size |
| `Empty content` | Page has no article tag | Check Docusaurus structure |
| `Collection not found` | First run on new cluster | Script creates it automatically |

### Logs

- Check console output for step-by-step progress
- Failed URLs are logged with HTTP status codes
- API errors include retry attempt information

## Time Estimate

| Book Size | Pages | Expected Duration |
|-----------|-------|-------------------|
| Small | 20-50 | 5-10 minutes |
| Medium | 50-100 | 10-20 minutes |
| Large | 100-200 | 20-40 minutes |

## Next Steps

After running the pipeline:

1. **Verify** embeddings are searchable with test queries
2. **Integrate** with your RAG application (out of scope for this feature)
3. **Re-run** pipeline when book content changes (idempotent)
