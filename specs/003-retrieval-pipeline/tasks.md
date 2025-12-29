# Tasks: Data Retrieval & Pipeline Validation

**Input**: Design documents from `/specs/003-retrieval-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel
- **[Story]**: US1, US2, US3
- Single file: `backend/retrieve.py`

---

## Phase 1: Setup

- [x] T001 Create `backend/retrieve.py` with imports and CONFIG dict from .env

---

## Phase 2: Foundational

- [x] T002 Add SearchResult and RetrievalResponse dataclasses in `backend/retrieve.py`
- [x] T003 Add log() and retry_with_backoff() utilities in `backend/retrieve.py`

**Checkpoint**: Base structure ready

---

## Phase 3: User Story 1 - Query Retrieval (P1) MVP

**Goal**: Retrieve relevant chunks via semantic search

**Test**: Run `uv run python retrieve.py "What is Physical AI?"` and verify results

- [x] T004 [US1] Implement embed_query() using Cohere in `backend/retrieve.py`
- [x] T005 [US1] Implement search_vectors() using Qdrant in `backend/retrieve.py`
- [x] T006 [US1] Implement retrieve() main function in `backend/retrieve.py`
- [x] T007 [US1] Implement format_results() for output in `backend/retrieve.py`
- [x] T008 [US1] Add CLI with argparse (query, --top-k, --threshold) in `backend/retrieve.py`

**Checkpoint**: Basic retrieval works

---

## Phase 4: User Story 2 - Pipeline Validation (P2)

**Goal**: Validate end-to-end pipeline with test queries

**Test**: Run `uv run python retrieve.py --validate` and verify 5/5 pass

- [x] T009 [US2] Add TEST_QUERIES list with 5 queries and expected keywords in `backend/retrieve.py`
- [x] T010 [US2] Implement validate_pipeline() function in `backend/retrieve.py`
- [x] T011 [US2] Add --validate flag to CLI in `backend/retrieve.py`

**Checkpoint**: Validation harness works

---

## Phase 5: User Story 3 - Error Handling (P3)

**Goal**: Handle errors gracefully with clear messages

**Test**: Run with empty query, verify error message returned

- [x] T012 [US3] Add validate_query() for empty/long query checks in `backend/retrieve.py`
- [x] T013 [US3] Add try/except blocks with descriptive errors in `backend/retrieve.py`
- [x] T014 [US3] Handle no-results case with clear message in `backend/retrieve.py`

**Checkpoint**: All error scenarios handled

---

## Phase 6: Polish

- [x] T015 Run full validation suite and verify 5/5 queries pass
- [x] T016 Test 3 manual queries and confirm <2s response time

---

## Dependencies

```
T001 → T002, T003 → T004-T008 (US1) → T009-T011 (US2) → T012-T014 (US3) → T015-T016
```

**Parallel**: T002 and T003 can run together. T004-T005 can run together.

---

## Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Setup | 1 | File creation |
| Foundational | 2 | Data classes, utilities |
| US1 (P1) | 5 | Core retrieval |
| US2 (P2) | 3 | Validation harness |
| US3 (P3) | 3 | Error handling |
| Polish | 2 | Final validation |

**Total**: 16 tasks | **MVP**: T001-T008 (8 tasks)
