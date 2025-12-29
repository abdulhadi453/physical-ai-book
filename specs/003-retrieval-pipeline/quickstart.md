# Quickstart: Data Retrieval & Pipeline Validation

**Branch**: `003-retrieval-pipeline` | **Date**: 2025-12-27

## Prerequisites

1. **Python 3.10+** (3.13 recommended, matching existing setup)
2. **Existing Qdrant collection** with embedded chunks from Spec 1
3. **API credentials** for Cohere and Qdrant Cloud

## Setup

### 1. Environment Configuration

Ensure `.env` file exists in `backend/` with required variables:

```bash
# Required - Cohere API
COHERE_API_KEY="your-cohere-api-key"

# Required - Qdrant Cloud
QDRANT_URL="https://your-cluster.cloud.qdrant.io:6333"
QDRANT_API_KEY="your-qdrant-api-key"
QDRANT_COLLECTION_NAME="Physical-AI-Book"
```

### 2. Dependencies

Using uv (already configured):

```bash
cd backend
uv sync  # Install from pyproject.toml
```

Required packages (already in pyproject.toml):
- `cohere>=5.20.1`
- `qdrant-client>=1.16.2`
- `python-dotenv>=1.2.1`

## Usage

### Basic Query

```bash
cd backend
uv run python retrieve.py "What is Physical AI?"
```

### With Options

```bash
# Custom number of results
uv run python retrieve.py "robot safety" --top-k 10

# Custom similarity threshold
uv run python retrieve.py "ROS 2 architecture" --threshold 0.6

# Combined options
uv run python retrieve.py "sim-to-real transfer" --top-k 3 --threshold 0.8
```

### Run Validation Suite

```bash
# Run all test queries
uv run python retrieve.py --validate

# Verbose validation output
uv run python retrieve.py --validate --verbose
```

## Expected Output

### Successful Query

```
[15:30:45] Query: "What is Physical AI?"
[15:30:46] Embedding query (1024 dimensions)...
[15:30:46] Searching Qdrant collection 'Physical-AI-Book'...
[15:30:47] Found 5 results (234ms)

Result 1 (score: 0.89):
  Source: Introduction to Physical AI
  URL: https://physical-ai-book.../intro
  Content: Physical AI refers to artificial intelligence systems
           that operate in and interact with the physical world...

Result 2 (score: 0.85):
  ...
```

### No Results

```
[15:30:45] Query: "quantum teleportation recipes"
[15:30:46] Embedding query (1024 dimensions)...
[15:30:46] Searching Qdrant collection 'Physical-AI-Book'...
[15:30:47] No results found above threshold 0.70

Status: no_matches
Message: No relevant content found for this query. Try rephrasing or broadening your search.
```

### Validation Output

```
[15:30:45] Running validation suite (5 test queries)...

Test 1: "What is Physical AI?"
  ✓ PASSED - 5 results, top score: 0.89
  Keywords found: physical, AI, systems

Test 2: "How do robots learn from demonstrations?"
  ✓ PASSED - 5 results, top score: 0.82
  Keywords found: robot, learning, demonstration

...

Validation Summary:
  Passed: 5/5
  Average top score: 0.85
  Average response time: 245ms
```

## Troubleshooting

### Connection Errors

```
Error: Qdrant connection failed
```
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Check network connectivity to Qdrant Cloud
- Ensure collection exists: run `main.py` embedding pipeline first

### Empty Results

```
Status: no_matches
```
- Lower the threshold: `--threshold 0.5`
- Verify collection has data: check Qdrant dashboard
- Ensure query relates to Physical AI/Robotics content

### Embedding Errors

```
Error: Cohere API unavailable
```
- Verify `COHERE_API_KEY` in `.env`
- Check Cohere API status: https://status.cohere.ai
- Wait for rate limit reset if exceeded

## File Structure

```
backend/
├── retrieve.py          # This implementation (NEW)
├── main.py              # Embedding pipeline (Spec 1)
├── .env                 # Environment configuration
├── pyproject.toml       # Dependencies
└── README.md            # Project documentation
```
