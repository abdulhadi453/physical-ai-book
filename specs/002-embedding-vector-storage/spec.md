# Feature Specification: Book Content Embedding and Vector Storage

**Feature Branch**: `002-embedding-vector-storage`
**Created**: 2025-12-24
**Status**: Draft
**Input**: User description: "Deploy book URLs, generate embeddings and store them in vector database"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Extract and Prepare Book Content (Priority: P1)

As an AI engineer, I need to crawl all published Docusaurus URLs from the deployed book site, extract clean text content, and prepare it for embedding generation so that the RAG pipeline has access to the complete, well-structured book content.

**Why this priority**: Without crawled content, there's nothing to embed or search. This is the foundational data layer that all other capabilities depend on. It represents the complete MVP - a working crawler that successfully extracts all book content.

**Independent Test**: Can be fully tested by running the crawler against the deployed Vercel URL and verifying that all pages are discovered, content is extracted without HTML artifacts, and output is saved in a structured format (JSON/text files).

**Acceptance Scenarios**:

1. **Given** a deployed Docusaurus book at a Vercel URL, **When** the crawler starts from the base URL, **Then** it discovers all linked pages within the book domain
2. **Given** discovered pages, **When** content is extracted, **Then** HTML tags, navigation elements, and boilerplate are removed, leaving only substantive text
3. **Given** extracted content, **When** saved to disk, **Then** each page is stored with its URL, title, and clean text in a structured format
4. **Given** crawling is complete, **When** reviewing the output, **Then** all main book sections, chapters, and sub-pages are present with no missing content

---

### User Story 2 - Generate and Store Vector Embeddings (Priority: P2)

As an AI engineer, I need to chunk the extracted book content into semantic units, generate vector embeddings using Cohere models, and store them in Qdrant cloud with appropriate metadata so that semantic search can retrieve relevant passages.

**Why this priority**: Once content is extracted (P1), this story enables the core RAG capability - semantic search. It transforms text into searchable vectors. This represents a standalone enhancement that can be tested independently by verifying embeddings are created and indexed.

**Independent Test**: Can be fully tested by providing sample extracted content from P1, running the embedding generation script, verifying Cohere API calls succeed, and confirming that vectors are successfully uploaded to Qdrant with searchable metadata (URL, title, chunk index).

**Acceptance Scenarios**:

1. **Given** extracted book content, **When** text is chunked, **Then** each chunk is between 200-1000 tokens with appropriate overlap for context preservation
2. **Given** text chunks, **When** embeddings are generated via Cohere API, **Then** each chunk produces a dense vector of expected dimensionality
3. **Given** generated embeddings, **When** uploaded to Qdrant cloud, **Then** vectors are stored with metadata including source URL, page title, chunk position, and original text
4. **Given** embeddings in Qdrant, **When** collection statistics are checked, **Then** the total vector count matches the number of chunks generated

---

### User Story 3 - Validate Vector Search Quality (Priority: P3)

As an AI engineer, I need to run test queries against the Qdrant collection and verify that semantically relevant chunks are returned in the correct order so that I can validate the RAG pipeline foundation is working correctly.

**Why this priority**: This validates the quality and utility of the entire pipeline (P1 + P2). While important, it's lower priority because it's a validation step rather than core functionality. The system can be considered "complete" without this, though less trustworthy.

**Independent Test**: Can be fully tested by defining 5-10 representative test queries (questions that the book should answer), running vector searches, and manually reviewing whether returned chunks contain relevant information and are ranked appropriately.

**Acceptance Scenarios**:

1. **Given** test queries about book topics, **When** vector search is performed, **Then** top-5 results contain chunks semantically related to the query
2. **Given** a specific factual question from the book, **When** searching, **Then** the chunk containing the answer appears in the top-3 results
3. **Given** diverse query types (factual, conceptual, how-to), **When** testing, **Then** at least 80% of queries return relevant results in the top-5
4. **Given** search results, **When** reviewing metadata, **Then** source URLs and page titles are correctly associated with returned chunks

---

### Edge Cases

- What happens when the deployed site is unreachable or returns 404/500 errors during crawling?
- How does the system handle pages with very little text content (< 50 words)?
- What happens when Cohere API rate limits are hit during batch embedding generation?
- How does the system handle duplicate pages or redirects during crawling?
- What happens when Qdrant cloud is unavailable during vector upload?
- How does the system handle pages with non-English content or special characters?
- What happens when a page's content exceeds the maximum context window for embeddings?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST crawl all publicly accessible URLs from the deployed Docusaurus book starting from the base Vercel URL
- **FR-002**: System MUST extract clean text content from each page, removing HTML tags, navigation menus, footers, and other non-content elements
- **FR-003**: System MUST preserve the page title, URL, and content hierarchy (if applicable) as metadata
- **FR-004**: System MUST chunk extracted text into semantic units suitable for embedding (typically 200-1000 tokens with overlap)
- **FR-005**: System MUST generate vector embeddings for each text chunk using Cohere embedding models
- **FR-006**: System MUST store generated embeddings in Qdrant cloud free tier with associated metadata (URL, title, chunk index, original text)
- **FR-007**: System MUST handle API failures gracefully with retry logic for both Cohere and Qdrant operations
- **FR-008**: System MUST support resumable operations (idempotent crawling and embedding generation)
- **FR-009**: System MUST provide configuration via environment variables for API keys, base URLs, and embedding parameters
- **FR-010**: System MUST log progress and errors to enable debugging and monitoring
- **FR-011**: System MUST validate that uploaded embeddings are searchable by running test vector queries
- **FR-012**: System MUST respect robots.txt and rate limits when crawling the deployed site
- **FR-013**: System MUST create or update a Qdrant collection with appropriate vector dimensions and distance metrics

### Key Entities

- **Book Page**: Represents a single page from the Docusaurus site with attributes including URL (string, unique identifier), title (string, extracted from page), raw content (HTML string), cleaned content (plain text with structure preserved), crawl timestamp (datetime), and page type (e.g., chapter, reference, tutorial).

- **Text Chunk**: Represents a semantic unit of text for embedding with attributes including chunk ID (unique identifier), source page URL (reference to Book Page), chunk index (integer, position within page), text content (string, 200-1000 tokens), token count (integer), overlap tokens (string, context from previous chunk), and chunk type (e.g., paragraph, code block, heading section).

- **Vector Embedding**: Represents the numerical vector representation of a text chunk with attributes including embedding ID (unique identifier), chunk reference (link to Text Chunk), vector values (array of floats, dimensionality based on Cohere model), embedding model (string, e.g., "embed-english-v3.0"), generation timestamp (datetime), and vector norm/magnitude (float, for validation).

- **Search Result**: Represents a query result from the vector database with attributes including query text (string), matching chunk ID (reference to Text Chunk), similarity score (float, 0-1 range), rank position (integer), source metadata (URL, title, chunk position), and retrieved at timestamp (datetime).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All pages from the deployed Docusaurus book are successfully crawled and extracted (100% coverage of accessible URLs)
- **SC-002**: Text extraction removes all HTML artifacts, navigation elements, and boilerplate, producing clean, readable content for 95%+ of pages
- **SC-003**: Embeddings are generated for all text chunks with a success rate of 99%+ (allowing for minor API transient failures with retries)
- **SC-004**: All generated embeddings are successfully stored in Qdrant cloud with correct metadata and are searchable
- **SC-005**: Test queries return relevant results with at least 80% of queries having correct content in the top-5 results
- **SC-006**: The entire pipeline (crawl, chunk, embed, store) completes within 60 minutes for a book with approximately 100-200 pages
- **SC-007**: The system can be re-run idempotently without creating duplicate entries or corrupting existing data
- **SC-008**: Vector search queries return results in under 500ms for single-query operations
- **SC-009**: Configuration and environment setup can be completed by a new developer in under 15 minutes following the documentation
- **SC-010**: Error logs clearly identify failure points (e.g., "Qdrant connection failed", "Cohere rate limit hit") with actionable error messages

## Assumptions *(mandatory)*

- The Docusaurus book is fully deployed and accessible via a public Vercel URL
- The deployed site uses standard Docusaurus routing and structure (no heavy customization that breaks standard crawling)
- Cohere API access is available with sufficient quota for embedding generation (assuming Cohere free tier or paid plan)
- Qdrant cloud free tier has sufficient capacity for the expected vector count (estimated 1000-5000 vectors for a typical book)
- Content is primarily in English (Cohere models used are English-optimized)
- The book structure is relatively stable and doesn't change frequently during the implementation period
- Network connectivity is stable enough for API calls to Cohere and Qdrant cloud
- Python 3.8+ is available as the runtime environment
- Standard Python libraries for web scraping (BeautifulSoup, requests) and vector operations are compatible with the environment

## Dependencies *(mandatory)*

### External Dependencies

- **Deployed Docusaurus Site**: The book must be live on Vercel with a stable URL; site must remain accessible during crawling; any downtime or deployment changes could interrupt the pipeline. **Owner**: DevOps/Deployment team
- **Cohere API**: Required for embedding generation; depends on API availability, rate limits, and quota. **Owner**: Cohere (third-party service)
- **Qdrant Cloud Free Tier**: Required for vector storage and search; depends on service uptime and free tier limits. **Owner**: Qdrant (third-party service)

### Internal Dependencies

- Configuration management system for storing API keys securely (environment variables or .env file)
- Python runtime environment with package management (pip/poetry/conda)

### Technical Constraints

- Qdrant cloud free tier limits: 1 cluster, specific vector count limits (typically 1M vectors, but verify current free tier specs)
- Cohere API rate limits: Dependent on plan tier, may require batching and retry logic
- Vercel bandwidth and request limits: Should not aggressively crawl to avoid rate limiting or IP bans
- Embedding model dimensionality: Must be consistent across all embeddings (e.g., Cohere embed-english-v3.0 produces 1024-dimensional vectors)

## Constraints *(mandatory)*

### In Scope

- Crawling all public URLs from the deployed Docusaurus book
- Extracting and cleaning text content from HTML pages
- Chunking text into semantic units optimized for embedding
- Generating embeddings using Cohere models
- Storing embeddings in Qdrant cloud with metadata
- Running test queries to validate vector search quality
- Modular Python scripts with clear separation of concerns (crawl, chunk, embed, store)
- Configuration via environment variables for API keys and parameters
- Basic error handling and logging for debugging
- Documentation for setup and execution

### Out of Scope

- Retrieval logic beyond basic vector search validation (no ranking, reranking, or relevance tuning)
- Agent or chatbot logic that uses the embeddings for question-answering
- FastAPI or web server integration for serving search results
- Frontend or UI for interacting with the vector database
- User authentication or authorization for accessing the vector database
- Analytics, monitoring, or observability dashboards
- Incremental updates or real-time synchronization when book content changes
- Multi-language support or translation of content
- Advanced chunking strategies (e.g., semantic chunking, sliding window optimizations)
- Evaluation metrics beyond basic top-k relevance checks
- Cost optimization or budget monitoring for API usage

### Non-Functional Requirements

- **Performance**: The pipeline should complete within 60 minutes for a typical book (100-200 pages)
- **Reliability**: Retry logic must handle transient API failures (3 retries with exponential backoff)
- **Maintainability**: Code must be modular with clear separation between crawling, chunking, embedding, and storage logic
- **Configurability**: All external endpoints, API keys, and parameters must be configurable via environment variables or config files
- **Idempotency**: Re-running the pipeline should not create duplicate embeddings or corrupt existing data

## Risks *(optional)*

- **Cohere API Rate Limits**: If the book is large, embedding generation may hit rate limits. **Mitigation**: Implement batching and exponential backoff retry logic; consider caching embeddings to avoid regeneration.
- **Qdrant Free Tier Limits**: If the book is very large (e.g., 500+ pages), the free tier may not accommodate all vectors. **Mitigation**: Monitor vector count; implement chunking strategies to reduce total vector count; consider upgrading to paid tier if needed.
- **Deployed Site Changes**: If the book is redeployed or URLs change during implementation, crawled data may become stale. **Mitigation**: Design crawler to be idempotent and easily re-runnable; version or timestamp crawled data.
- **Embedding Quality Issues**: Poor text chunking or cleaning may result in low-quality embeddings that don't retrieve relevant content. **Mitigation**: Implement robust HTML cleaning; test chunking strategies with sample data before full pipeline run; include manual review of test query results.
- **Network/API Downtime**: Cohere or Qdrant outages could block the pipeline. **Mitigation**: Implement robust error handling and logging; design pipeline to resume from last successful step.

## Open Questions *(optional)*

None at this time. All critical decisions have reasonable defaults based on the constraints provided (Python, Cohere, Qdrant, Vercel-deployed Docusaurus book).
