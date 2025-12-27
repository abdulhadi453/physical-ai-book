#!/usr/bin/env python3
"""
Book Content Embedding Pipeline

Crawls a Docusaurus book, extracts content, generates embeddings with Cohere,
and stores them in Qdrant for semantic search.

Usage:
    uv run python main.py
"""

# ============================================================
# Section 1: Imports and Configuration
# ============================================================

import os
import hashlib
import time
from datetime import datetime
from dataclasses import dataclass
from typing import Optional
from urllib.parse import urljoin, urlparse

from dotenv import load_dotenv
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

# Load environment variables
load_dotenv()

# Configuration
CONFIG = {
    "cohere_api_key": os.getenv("COHERE_API_KEY"),
    "qdrant_url": os.getenv("QDRANT_URL"),
    "qdrant_api_key": os.getenv("QDRANT_API_KEY"),
    "collection_name": os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings"),
    "book_base_url": os.getenv("BOOK_BASE_URL"),
    "embedding_model": "embed-english-v3.0",
    "vector_size": 1024,
    "chunk_size": 2000,  # ~500 tokens (4 chars per token)
    "chunk_overlap": 400,  # ~100 tokens overlap
    "batch_size": 20,  # Reduced for Cohere trial rate limits
    "max_retries": 3,
    "retry_delay": 1,
}


# ============================================================
# Section 2: Data Classes
# ============================================================


@dataclass
class BookPage:
    """A page extracted from the Docusaurus book."""

    url: str
    title: str
    content: str
    crawled_at: datetime


@dataclass
class TextChunk:
    """A chunk of text for embedding."""

    chunk_id: str
    source_url: str
    source_title: str
    chunk_index: int
    content: str
    token_count: int


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
            delay = base_delay * (2**attempt)
            log(f"  Retry {attempt + 1}/{max_retries} after {delay}s: {e}")
            time.sleep(delay)


def estimate_tokens(text: str) -> int:
    """Estimate token count (roughly 4 characters per token)."""
    return len(text) // 4


def generate_chunk_id(url: str, chunk_index: int) -> str:
    """Generate a deterministic chunk ID for idempotency."""
    return hashlib.md5(f"{url}_{chunk_index}".encode()).hexdigest()


# ============================================================
# Section 4: URL Discovery and Crawling
# ============================================================


def discover_urls(base_url: str) -> list[str]:
    """Discover all documentation URLs from the site."""
    log(f"Discovering URLs from {base_url}")
    visited = set()
    to_visit = [base_url]
    doc_urls = []

    parsed_base = urlparse(base_url)
    base_domain = parsed_base.netloc

    while to_visit:
        url = to_visit.pop(0)

        if url in visited:
            continue
        visited.add(url)

        try:
            response = requests.get(url, timeout=10)
            if response.status_code != 200:
                continue

            soup = BeautifulSoup(response.text, "lxml")

            # Check if this is a content page (has article or main content)
            if soup.find("article") or soup.find("main"):
                doc_urls.append(url)

            # Find all links
            for link in soup.find_all("a", href=True):
                href = link["href"]
                full_url = urljoin(url, href)
                parsed = urlparse(full_url)

                # Only follow links on the same domain
                if parsed.netloc != base_domain:
                    continue

                # Skip anchors, external links, and non-doc pages
                if "#" in full_url:
                    full_url = full_url.split("#")[0]

                # Skip common non-content paths
                skip_patterns = [
                    "/search",
                    "/tags",
                    "/blog",
                    "/api/",
                    ".xml",
                    ".json",
                    ".css",
                    ".js",
                ]
                if any(pattern in full_url.lower() for pattern in skip_patterns):
                    continue

                if full_url not in visited and full_url not in to_visit:
                    to_visit.append(full_url)

        except Exception as e:
            log(f"  Error fetching {url}: {e}")

    log(f"  Found {len(doc_urls)} content pages")
    return list(set(doc_urls))


def fetch_page(url: str) -> Optional[str]:
    """Fetch a page with retry logic."""

    def _fetch():
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        return response.text

    try:
        return retry_with_backoff(_fetch, CONFIG["max_retries"], CONFIG["retry_delay"])
    except Exception as e:
        log(f"  Failed to fetch {url}: {e}")
        return None


def clean_html(soup: BeautifulSoup) -> BeautifulSoup:
    """Remove non-content elements from HTML."""
    # Remove navigation, footer, sidebar, scripts, styles
    for tag in soup.find_all(
        ["nav", "footer", "aside", "script", "style", "noscript"]
    ):
        tag.decompose()

    # Remove elements by common class names (but preserve article content)
    for class_name in [
        "navbar",
        "sidebar__menu",
        "table-of-contents",
        "breadcrumbs",
        "pagination-nav",
        "theme-doc-footer",
    ]:
        for tag in soup.find_all(class_=lambda x: x and class_name in str(x).lower()):
            # Don't remove if it's an article or contains main content
            if tag.name not in ["article", "main", "section"]:
                tag.decompose()

    return soup


def extract_content(html: str) -> tuple[str, str]:
    """Extract title and clean text content from HTML."""
    soup = BeautifulSoup(html, "lxml")

    # Extract title
    title = ""
    h1 = soup.find("h1")
    if h1:
        title = h1.get_text(strip=True)
    elif soup.title:
        title = soup.title.get_text(strip=True)

    # Get main content BEFORE cleaning (Docusaurus specific selectors)
    main_content = None

    # Try Docusaurus-specific selectors first
    main_content = soup.find("article")
    if not main_content:
        main_content = soup.find("div", class_=lambda x: x and "markdown" in str(x).lower())
    if not main_content:
        main_content = soup.find("main")
    if not main_content:
        main_content = soup.find("div", {"role": "main"})
    if not main_content:
        main_content = soup.find("body")

    if not main_content:
        return title, ""

    # Clean the main content area
    main_content = clean_html(main_content)

    # Extract text with basic structure preservation
    text = main_content.get_text(separator="\n", strip=True)

    # Clean up excessive whitespace
    lines = [line.strip() for line in text.split("\n") if line.strip()]
    content = "\n".join(lines)

    return title, content


def crawl_site(base_url: str) -> list[BookPage]:
    """Crawl the site and extract all pages."""
    log("Starting site crawl...")
    urls = discover_urls(base_url)
    pages = []

    for i, url in enumerate(urls, 1):
        log(f"  Processing page {i}/{len(urls)}: {url}")
        html = fetch_page(url)
        if not html:
            continue

        title, content = extract_content(html)
        if not content or len(content) < 50:  # Skip very short pages
            log(f"    Skipped (too short)")
            continue

        pages.append(
            BookPage(url=url, title=title or url, content=content, crawled_at=datetime.now())
        )

    log(f"Crawl complete: {len(pages)} pages extracted")
    return pages


# ============================================================
# Section 5: Text Chunking
# ============================================================


def chunk_text(content: str, url: str, title: str) -> list[TextChunk]:
    """Split content into chunks with overlap."""
    chunks = []
    chunk_size = CONFIG["chunk_size"]
    overlap = CONFIG["chunk_overlap"]

    # Split by paragraphs first, then recombine
    paragraphs = content.split("\n")
    current_chunk = ""
    chunk_index = 0

    for para in paragraphs:
        if len(current_chunk) + len(para) + 1 <= chunk_size:
            current_chunk += para + "\n"
        else:
            if current_chunk.strip():
                chunks.append(
                    TextChunk(
                        chunk_id=generate_chunk_id(url, chunk_index),
                        source_url=url,
                        source_title=title,
                        chunk_index=chunk_index,
                        content=current_chunk.strip(),
                        token_count=estimate_tokens(current_chunk),
                    )
                )
                chunk_index += 1

            # Start new chunk with overlap from previous
            if overlap > 0 and current_chunk:
                overlap_text = current_chunk[-overlap:]
                current_chunk = overlap_text + para + "\n"
            else:
                current_chunk = para + "\n"

    # Don't forget the last chunk
    if current_chunk.strip():
        chunks.append(
            TextChunk(
                chunk_id=generate_chunk_id(url, chunk_index),
                source_url=url,
                source_title=title,
                chunk_index=chunk_index,
                content=current_chunk.strip(),
                token_count=estimate_tokens(current_chunk),
            )
        )

    return chunks


# ============================================================
# Section 6: Embedding Generation
# ============================================================


def batch_embed(
    texts: list[str], cohere_client: cohere.Client
) -> list[list[float]]:
    """Generate embeddings for a batch of texts."""

    def _embed():
        response = cohere_client.embed(
            model=CONFIG["embedding_model"],
            texts=texts,
            input_type="search_document",
            truncate="END",
        )
        return response.embeddings

    return retry_with_backoff(_embed, CONFIG["max_retries"], CONFIG["retry_delay"])


def generate_embeddings(
    chunks: list[TextChunk], cohere_client: cohere.Client
) -> list[tuple[TextChunk, list[float]]]:
    """Generate embeddings for all chunks."""
    log(f"Generating embeddings for {len(chunks)} chunks...")
    results = []
    batch_size = CONFIG["batch_size"]

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i : i + batch_size]
        batch_num = i // batch_size + 1
        total_batches = (len(chunks) + batch_size - 1) // batch_size
        log(f"  Embedding batch {batch_num}/{total_batches} ({len(batch)} chunks)")

        texts = [chunk.content for chunk in batch]
        embeddings = batch_embed(texts, cohere_client)

        for chunk, embedding in zip(batch, embeddings):
            results.append((chunk, embedding))

        # Add delay between batches to respect rate limits
        if i + batch_size < len(chunks):
            time.sleep(15)  # 15 second delay between batches for trial API

    log(f"  Generated {len(results)} embeddings")
    return results


# ============================================================
# Section 7: Qdrant Storage
# ============================================================


def setup_collection(client: QdrantClient, collection_name: str) -> None:
    """Create or recreate the Qdrant collection."""
    log(f"Setting up collection '{collection_name}'...")

    # Check if collection exists
    collections = client.get_collections().collections
    exists = any(c.name == collection_name for c in collections)

    if exists:
        log(f"  Collection exists, recreating...")
        client.delete_collection(collection_name)

    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=CONFIG["vector_size"], distance=Distance.COSINE
        ),
    )
    log(f"  Collection '{collection_name}' ready")


def store_embeddings(
    client: QdrantClient,
    collection_name: str,
    chunk_vectors: list[tuple[TextChunk, list[float]]],
) -> None:
    """Store embeddings in Qdrant."""
    log(f"Storing {len(chunk_vectors)} vectors in Qdrant...")

    points = []
    for chunk, vector in chunk_vectors:
        points.append(
            PointStruct(
                id=chunk.chunk_id,
                vector=vector,
                payload={
                    "url": chunk.source_url,
                    "title": chunk.source_title,
                    "chunk_index": chunk.chunk_index,
                    "content": chunk.content,
                    "token_count": chunk.token_count,
                    "embedded_at": datetime.now().isoformat(),
                },
            )
        )

    # Upsert in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i : i + batch_size]

        def _upsert():
            client.upsert(collection_name=collection_name, points=batch)

        retry_with_backoff(_upsert, CONFIG["max_retries"], CONFIG["retry_delay"])

    log(f"  Stored {len(points)} vectors successfully")


# ============================================================
# Section 8: Validation
# ============================================================


def embed_query(query: str, cohere_client: cohere.Client) -> list[float]:
    """Generate embedding for a search query."""

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
    client: QdrantClient, collection_name: str, query_vector: list[float], limit: int = 5
) -> list[dict]:
    """Search for similar vectors."""
    results = client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=limit,
        with_payload=True,
    )

    return [
        {
            "score": hit.score,
            "title": hit.payload.get("title", ""),
            "url": hit.payload.get("url", ""),
            "content": hit.payload.get("content", "")[:200] + "...",
        }
        for hit in results.points
    ]


def validate_search(
    qdrant_client: QdrantClient, cohere_client: cohere.Client, collection_name: str
) -> None:
    """Run test queries to validate search quality."""
    log("Running validation queries...")

    test_queries = [
        "What is physical AI?",
        "How do robots perceive their environment?",
        "What is ROS 2?",
        "How does simulation work in robotics?",
        "What are humanoid robots?",
    ]

    passed = 0
    for query in test_queries:
        log(f"  Query: '{query}'")
        query_vector = embed_query(query, cohere_client)
        results = search_vectors(qdrant_client, collection_name, query_vector, limit=3)

        if results:
            passed += 1
            for i, r in enumerate(results, 1):
                log(f"    {i}. (score: {r['score']:.3f}) {r['title']}")
        else:
            log(f"    No results found")

    log(f"Validation: {passed}/{len(test_queries)} queries returned results")


# ============================================================
# Section 9: Main Orchestration
# ============================================================


def validate_config() -> bool:
    """Validate required configuration."""
    required = ["cohere_api_key", "qdrant_url", "qdrant_api_key", "book_base_url"]
    missing = [key for key in required if not CONFIG.get(key)]

    if missing:
        log(f"ERROR: Missing required configuration: {', '.join(missing)}")
        log("Please set these in your .env file")
        return False
    return True


def main() -> None:
    """Main pipeline orchestration."""
    start_time = time.time()
    log("=" * 60)
    log("Book Content Embedding Pipeline")
    log("=" * 60)

    # Validate configuration
    if not validate_config():
        return

    # Initialize clients
    log("Initializing clients...")
    cohere_client = cohere.Client(api_key=CONFIG["cohere_api_key"])
    qdrant_client = QdrantClient(
        url=CONFIG["qdrant_url"], api_key=CONFIG["qdrant_api_key"]
    )

    # Step 1: Crawl site
    log("\n[Step 1/5] Crawling book content...")
    pages = crawl_site(CONFIG["book_base_url"])
    if not pages:
        log("ERROR: No pages extracted. Check the book URL.")
        return

    # Step 2: Chunk content
    log("\n[Step 2/5] Chunking content...")
    all_chunks = []
    for page in pages:
        chunks = chunk_text(page.content, page.url, page.title)
        all_chunks.extend(chunks)
    log(f"  Created {len(all_chunks)} chunks from {len(pages)} pages")

    # Step 3: Generate embeddings
    log("\n[Step 3/5] Generating embeddings...")
    chunk_vectors = generate_embeddings(all_chunks, cohere_client)

    # Step 4: Store in Qdrant
    log("\n[Step 4/5] Storing in Qdrant...")
    setup_collection(qdrant_client, CONFIG["collection_name"])
    store_embeddings(qdrant_client, CONFIG["collection_name"], chunk_vectors)

    # Step 5: Validate
    log("\n[Step 5/5] Validating search quality...")
    validate_search(qdrant_client, cohere_client, CONFIG["collection_name"])

    # Summary
    elapsed = time.time() - start_time
    log("\n" + "=" * 60)
    log("Pipeline Complete!")
    log("=" * 60)
    log(f"  Pages crawled: {len(pages)}")
    log(f"  Chunks created: {len(all_chunks)}")
    log(f"  Vectors stored: {len(chunk_vectors)}")
    log(f"  Collection: {CONFIG['collection_name']}")
    log(f"  Time elapsed: {elapsed:.1f}s")


if __name__ == "__main__":
    main()
