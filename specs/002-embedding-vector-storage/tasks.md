# Tasks: Book Content Embedding and Vector Storage

**Input**: Design documents from `/specs/002-embedding-vector-storage/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Manual validation only (no pytest framework per plan.md)

**Organization**: Tasks grouped by user story for independent implementation

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1, US2, US3 (maps to spec.md user stories)
- All code in single `backend/main.py` file

---

## Phase 1: Setup

**Purpose**: Initialize backend project with uv

- [x] T001 Create `backend/` directory at repository root
- [x] T002 Run `uv init` inside `backend/` to create pyproject.toml
- [x] T003 Add dependencies: `uv add cohere qdrant-client beautifulsoup4 requests python-dotenv`
- [x] T004 [P] Create `backend/.env.example` with template for COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, BOOK_BASE_URL, QDRANT_COLLECTION_NAME
- [x] T005 [P] Create `backend/.gitignore` with .env, __pycache__, .venv entries

**Checkpoint**: Project initialized, dependencies installed

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core structure in main.py that all user stories depend on

- [x] T006 Create `backend/main.py` with imports section (os, hashlib, datetime, time, dataclasses, typing, dotenv, requests, bs4, cohere, qdrant_client)
- [x] T007 Add environment loading with load_dotenv() and config dict in `backend/main.py`
- [x] T008 Add BookPage and TextChunk dataclasses in `backend/main.py`
- [x] T009 Add retry_with_backoff() helper function (3 retries, exponential backoff) in `backend/main.py`
- [x] T010 Add logging setup (print-based progress messages) in `backend/main.py`

**Checkpoint**: Foundation ready - user story implementation can begin

---

## Phase 3: User Story 1 - Extract and Prepare Book Content (Priority: P1)

**Goal**: Crawl Docusaurus site and extract clean text content

**Independent Test**: Run `uv run python main.py` with BOOK_BASE_URL set; verify pages are discovered and content extracted

### Implementation for User Story 1

- [x] T011 [US1] Implement `discover_urls(base_url)` - find all doc links from sitemap or link traversal in `backend/main.py`
- [x] T012 [US1] Implement `fetch_page(url)` - HTTP GET with retry logic in `backend/main.py`
- [x] T013 [US1] Implement `extract_content(html)` - parse HTML, return (title, clean_text) in `backend/main.py`
- [x] T014 [US1] Implement `clean_html(soup)` - remove nav, footer, script, style elements in `backend/main.py`
- [x] T015 [US1] Implement `crawl_site(base_url)` - orchestrate discovery, fetch, extract; return List[BookPage] in `backend/main.py`
- [x] T016 [US1] Add progress logging for crawl operations (pages found, pages processed) in `backend/main.py`

**Checkpoint**: US1 complete - can crawl and extract content independently

---

## Phase 4: User Story 2 - Generate and Store Vector Embeddings (Priority: P2)

**Goal**: Chunk text, generate embeddings with Cohere, store in Qdrant

**Independent Test**: After US1, run pipeline; verify vectors appear in Qdrant collection

### Implementation for User Story 2

- [x] T017 [US2] Implement `estimate_tokens(text)` - return chars // 4 as token estimate in `backend/main.py`
- [x] T018 [US2] Implement `chunk_text(content, url, title)` - split into 500-token chunks with 100-token overlap, return List[TextChunk] in `backend/main.py`
- [x] T019 [US2] Implement `generate_chunk_id(url, chunk_index)` - MD5 hash for idempotency in `backend/main.py`
- [x] T020 [US2] Implement `batch_embed(texts, cohere_client)` - call Cohere API with batches of 96 in `backend/main.py`
- [x] T021 [US2] Implement `generate_embeddings(chunks, cohere_client)` - orchestrate batching, return List[Tuple[TextChunk, vector]] in `backend/main.py`
- [x] T022 [US2] Implement `setup_collection(qdrant_client, collection_name)` - create collection with 1024 dims, cosine distance in `backend/main.py`
- [x] T023 [US2] Implement `store_embeddings(qdrant_client, collection_name, chunk_vectors)` - upsert points with payload in `backend/main.py`
- [x] T024 [US2] Add progress logging for embedding and storage operations in `backend/main.py`

**Checkpoint**: US2 complete - can chunk, embed, and store vectors

---

## Phase 5: User Story 3 - Validate Vector Search Quality (Priority: P3)

**Goal**: Run test queries to verify semantic search works

**Independent Test**: Run validation; verify top-5 results contain relevant content

### Implementation for User Story 3

- [x] T025 [US3] Implement `embed_query(query, cohere_client)` - generate query embedding with input_type="search_query" in `backend/main.py`
- [x] T026 [US3] Implement `search_vectors(qdrant_client, collection_name, query_vector, limit)` - return top-k results in `backend/main.py`
- [x] T027 [US3] Implement `validate_search(qdrant_client, cohere_client, collection_name)` - run 3-5 test queries, print results in `backend/main.py`
- [x] T028 [US3] Add sample test queries relevant to Physical AI book content in `backend/main.py`

**Checkpoint**: US3 complete - validation confirms search quality

---

## Phase 6: Main Orchestration

**Purpose**: Wire everything together in main() function

- [x] T029 Implement `main()` function - orchestrate: load config → crawl → chunk → embed → store → validate in `backend/main.py`
- [x] T030 Add `if __name__ == "__main__": main()` entry point in `backend/main.py`
- [x] T031 Add overall pipeline timing and summary output in `backend/main.py`

**Checkpoint**: Pipeline fully functional end-to-end

---

## Phase 7: Polish

**Purpose**: Final cleanup and documentation

- [x] T032 [P] Verify .env.example has all required variables with comments
- [x] T033 Run full pipeline against live book URL and verify results
- [x] T034 Update quickstart.md if any steps changed during implementation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Phase 1
- **US1 (Phase 3)**: Depends on Phase 2
- **US2 (Phase 4)**: Depends on Phase 2 (can start parallel to US1 but logically follows)
- **US3 (Phase 5)**: Depends on Phase 2 (needs embeddings from US2 to validate)
- **Orchestration (Phase 6)**: Depends on US1, US2, US3
- **Polish (Phase 7)**: Depends on Phase 6

### Logical Flow

```
Setup → Foundational → US1 (crawl) → US2 (embed) → US3 (validate) → Orchestration → Polish
```

### Parallel Opportunities

Within Setup:
- T004 and T005 can run in parallel

Within each User Story:
- Tasks within US1 are sequential (each builds on previous)
- Tasks within US2 are sequential
- Tasks within US3 are sequential

---

## Implementation Strategy

### MVP (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: US1 (crawling)
4. **STOP**: Test crawling works independently

### Full Pipeline

1. Complete all phases sequentially
2. Test after each user story checkpoint
3. Final validation with live book URL

---

## Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Setup | T001-T005 | Project init with uv |
| Foundational | T006-T010 | Core structure |
| US1 (P1) | T011-T016 | Crawl and extract |
| US2 (P2) | T017-T024 | Chunk, embed, store |
| US3 (P3) | T025-T028 | Validate search |
| Orchestration | T029-T031 | Wire together |
| Polish | T032-T034 | Cleanup |

**Total Tasks**: 34
**MVP Tasks**: 16 (through US1)
**Single File**: All code in `backend/main.py`
