#!/usr/bin/env python3
"""
Retrieval Pipeline for Physical AI Book

Queries the Qdrant vector database with semantic search to retrieve
relevant book content chunks for RAG applications.

Usage:
    uv run python retrieve.py "What is Physical AI?"
    uv run python retrieve.py "robot safety" --top-k 10
    uv run python retrieve.py --validate
"""

# ============================================================
# Section 1: Imports and Configuration
# ============================================================

import os
import time
import argparse
from datetime import datetime
from dataclasses import dataclass

from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Configuration
CONFIG = {
    "cohere_api_key": os.getenv("COHERE_API_KEY"),
    "qdrant_url": os.getenv("QDRANT_URL"),
    "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
    "collection_name": os.getenv("QDRANT_COLLECTION_NAME", "Physical-AI-Book"),
    "embedding_model": "embed-english-v3.0",
    "vector_size": 1024,
    "default_top_k": 5,
    "default_threshold": 0.5,
    "max_retries": 3,
    "retry_delay": 1.0,
    "max_query_length": 8000,
}


# ============================================================
# Section 2: Data Classes
# ============================================================


@dataclass
class SearchResult:
    """A single search result from the vector database."""
    chunk_id: str
    content: str
    source_url: str
    source_title: str
    chunk_index: int
    token_count: int
    similarity_score: float
    embedded_at: str


@dataclass
class RetrievalResponse:
    """Response wrapper for retrieval operations."""
    query: str
    results: list[SearchResult]
    total_found: int
    search_time_ms: float
    status: str  # "success", "no_matches", "error"
    message: str


# ============================================================
# Section 3: Utility Functions
# ============================================================


def log(message: str) -> None:
    """Print a timestamped log message."""
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {message}")


def retry_with_backoff(func, max_retries: int = 3, base_delay: float = 1.0):
    """Execute a function with exponential backoff retry."""
    for attempt in range(max_retries):
        try:
            return func()
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            delay = base_delay * (2 ** attempt)
            log(f"  Retry {attempt + 1}/{max_retries} after {delay}s: {e}")
            time.sleep(delay)


def validate_query(query: str) -> tuple[bool, str]:
    """Validate query input. Returns (is_valid, error_message)."""
    if not query or not query.strip():
        return False, "Query cannot be empty"
    if len(query) > CONFIG["max_query_length"]:
        return False, f"Query exceeds {CONFIG['max_query_length']} character limit"
    return True, ""


# ============================================================
# Section 4: Core Functions
# ============================================================


def embed_query(query: str, cohere_client: cohere.Client) -> list[float]:
    """Generate embedding for a search query using Cohere."""
    def _embed():
        response = cohere_client.embed(
            model=CONFIG["embedding_model"],
            texts=[query],
            input_type="search_query",
            truncate="END",
        )
        return response.embeddings[0]

    return retry_with_backoff(_embed, CONFIG["max_retries"], CONFIG["retry_delay"])


def search_vectors(
    client: QdrantClient,
    query_vector: list[float],
    limit: int = 5,
    score_threshold: float = 0.7
) -> list[dict]:
    """Search Qdrant for similar vectors."""
    results = client.query_points(
        collection_name=CONFIG["collection_name"],
        query=query_vector,
        limit=limit,
        with_payload=True,
        score_threshold=score_threshold,
    )

    return [
        {
            "id": hit.id,
            "score": hit.score,
            "title": hit.payload.get("title", ""),
            "url": hit.payload.get("url", ""),
            "content": hit.payload.get("content", ""),
            "chunk_index": hit.payload.get("chunk_index", 0),
            "token_count": hit.payload.get("token_count", 0),
            "embedded_at": hit.payload.get("embedded_at", ""),
        }
        for hit in results.points
    ]


def retrieve(
    query: str,
    cohere_client: cohere.Client,
    qdrant_client: QdrantClient,
    top_k: int = 5,
    threshold: float = 0.7
) -> RetrievalResponse:
    """Main retrieval function: query -> embed -> search -> format."""
    start_time = time.time()

    # Validate query
    is_valid, error_msg = validate_query(query)
    if not is_valid:
        return RetrievalResponse(
            query=query,
            results=[],
            total_found=0,
            search_time_ms=0,
            status="error",
            message=error_msg
        )

    try:
        # Generate embedding
        log(f"Embedding query ({CONFIG['vector_size']} dimensions)...")
        query_vector = embed_query(query, cohere_client)

        # Search Qdrant
        log(f"Searching Qdrant collection '{CONFIG['collection_name']}'...")
        raw_results = search_vectors(qdrant_client, query_vector, top_k, threshold)

        # Format results
        results = [
            SearchResult(
                chunk_id=str(r["id"]),
                content=r["content"],
                source_url=r["url"],
                source_title=r["title"],
                chunk_index=r["chunk_index"],
                token_count=r["token_count"],
                similarity_score=r["score"],
                embedded_at=r["embedded_at"]
            )
            for r in raw_results
        ]

        elapsed_ms = (time.time() - start_time) * 1000

        if not results:
            return RetrievalResponse(
                query=query,
                results=[],
                total_found=0,
                search_time_ms=elapsed_ms,
                status="no_matches",
                message=f"No relevant results found above threshold {threshold}"
            )

        return RetrievalResponse(
            query=query,
            results=results,
            total_found=len(results),
            search_time_ms=elapsed_ms,
            status="success",
            message=f"Found {len(results)} results"
        )

    except Exception as e:
        elapsed_ms = (time.time() - start_time) * 1000
        error_type = type(e).__name__
        if "cohere" in str(type(e).__module__).lower():
            message = "Embedding service unavailable, try again later"
        elif "qdrant" in str(type(e).__module__).lower():
            message = "Vector database connection failed"
        else:
            message = f"Retrieval failed: {error_type}"

        return RetrievalResponse(
            query=query,
            results=[],
            total_found=0,
            search_time_ms=elapsed_ms,
            status="error",
            message=message
        )


def safe_print(text: str) -> None:
    """Print text safely, handling encoding issues."""
    try:
        print(text)
    except UnicodeEncodeError:
        # Replace problematic characters with ?
        print(text.encode('ascii', 'replace').decode('ascii'))


def format_results(response: RetrievalResponse) -> None:
    """Format and print retrieval results."""
    log(f"Found {response.total_found} results ({response.search_time_ms:.0f}ms)")
    print()

    if response.status == "error":
        print(f"Error: {response.message}")
        return

    if response.status == "no_matches":
        print(f"Status: {response.status}")
        print(f"Message: {response.message}")
        return

    for i, result in enumerate(response.results, 1):
        print(f"Result {i} (score: {result.similarity_score:.2f}):")
        safe_print(f"  Source: {result.source_title}")
        print(f"  URL: {result.source_url}")
        # Clean content for display
        content_preview = result.content[:300].encode('ascii', 'replace').decode('ascii')
        print(f"  Content: {content_preview}...")
        print()


# ============================================================
# Section 5: Validation Harness
# ============================================================


TEST_QUERIES = [
    {
        "query": "What is Physical AI?",
        "keywords": ["physical", "ai", "robot"],
        "purpose": "Core concept"
    },
    {
        "query": "How do robots learn from demonstrations?",
        "keywords": ["learn", "demonstration", "imitation"],
        "purpose": "VLA/learning"
    },
    {
        "query": "Robot safety considerations",
        "keywords": ["safety", "risk"],
        "purpose": "Ethics"
    },
    {
        "query": "ROS 2 architecture",
        "keywords": ["ros", "node"],
        "purpose": "Framework"
    },
    {
        "query": "Sim-to-real transfer",
        "keywords": ["simulation", "real", "transfer"],
        "purpose": "Deployment"
    },
]


def check_keywords(content: str, keywords: list[str]) -> list[str]:
    """Check which keywords appear in content."""
    content_lower = content.lower()
    return [kw for kw in keywords if kw.lower() in content_lower]


def validate_pipeline(cohere_client: cohere.Client, qdrant_client: QdrantClient, verbose: bool = False) -> dict:
    """Run validation test suite."""
    log(f"Running validation suite ({len(TEST_QUERIES)} test queries)...")
    print()

    results = []
    total_passed = 0

    for i, test in enumerate(TEST_QUERIES, 1):
        query = test["query"]
        expected_keywords = test["keywords"]

        start = time.time()
        response = retrieve(query, cohere_client, qdrant_client, top_k=5, threshold=0.5)
        elapsed = (time.time() - start) * 1000

        # Check if keywords found in results
        all_content = " ".join([r.content for r in response.results])
        found_keywords = check_keywords(all_content, expected_keywords)

        # Pass if we got results with at least one keyword
        passed = response.total_found > 0 and len(found_keywords) > 0
        if passed:
            total_passed += 1

        top_score = response.results[0].similarity_score if response.results else 0

        result = {
            "query": query,
            "expected_keywords": expected_keywords,
            "passed": passed,
            "results_count": response.total_found,
            "top_result_score": top_score,
            "keywords_found": found_keywords,
            "execution_time_ms": elapsed
        }
        results.append(result)

        # Print per-test output
        status = "PASSED" if passed else "FAILED"
        mark = "[PASS]" if passed else "[FAIL]"
        print(f"Test {i}: \"{query}\"")
        print(f"  {mark} {status} - {response.total_found} results, top score: {top_score:.2f}")
        print(f"  Keywords found: {', '.join(found_keywords) if found_keywords else 'none'}")
        print()

    # Summary
    avg_score = sum(r["top_result_score"] for r in results) / len(results) if results else 0
    avg_time = sum(r["execution_time_ms"] for r in results) / len(results) if results else 0

    print("=" * 50)
    print("Validation Summary:")
    print(f"  Passed: {total_passed}/{len(TEST_QUERIES)}")
    print(f"  Average top score: {avg_score:.2f}")
    print(f"  Average response time: {avg_time:.0f}ms")
    print("=" * 50)

    return {
        "total_tests": len(TEST_QUERIES),
        "passed": total_passed,
        "failed": len(TEST_QUERIES) - total_passed,
        "results": results,
        "average_score": avg_score,
        "average_time_ms": avg_time,
        "status": "pass" if total_passed == len(TEST_QUERIES) else "fail"
    }


# ============================================================
# Section 6: CLI Interface
# ============================================================


def validate_config() -> bool:
    """Validate required configuration."""
    required = ["cohere_api_key", "qdrant_url", "qdrant_api_key"]
    missing = [key for key in required if not CONFIG.get(key)]

    if missing:
        log(f"ERROR: Missing required configuration: {', '.join(missing)}")
        log("Please set these in your .env file")
        return False
    return True


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Retrieve relevant chunks from Physical AI book content."
    )
    parser.add_argument("query", nargs="?", help="Natural language search query")
    parser.add_argument("--top-k", type=int, default=CONFIG["default_top_k"],
                        help=f"Number of results (default: {CONFIG['default_top_k']})")
    parser.add_argument("--threshold", type=float, default=CONFIG["default_threshold"],
                        help=f"Minimum similarity score (default: {CONFIG['default_threshold']})")
    parser.add_argument("--validate", action="store_true",
                        help="Run validation test suite")
    parser.add_argument("--verbose", action="store_true",
                        help="Verbose output for validation")

    args = parser.parse_args()

    # Validate config
    if not validate_config():
        return

    # Initialize clients
    log("Initializing clients...")
    cohere_client = cohere.Client(api_key=CONFIG["cohere_api_key"])
    qdrant_client = QdrantClient(
        url=CONFIG["qdrant_url"],
        api_key=CONFIG["qdrant_api_key"]
    )

    # Run validation or query
    if args.validate:
        validate_pipeline(cohere_client, qdrant_client, args.verbose)
    elif args.query:
        log(f"Query: \"{args.query}\"")
        response = retrieve(
            args.query,
            cohere_client,
            qdrant_client,
            top_k=args.top_k,
            threshold=args.threshold
        )
        format_results(response)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
