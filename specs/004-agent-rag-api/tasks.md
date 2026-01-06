# Tasks: Agent-Based RAG Backend with FastAPI

**Input**: Design documents from `/specs/004-agent-rag-api/`
**Prerequisites**: plan.md (required), spec.md (required)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create requirements.txt with FastAPI, OpenAI Agents SDK, Qdrant client, Pydantic, python-dotenv, uvicorn
- [x] T002 Create .env.example with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME variables
- [x] T003 Create tests/__init__.py to make tests a package
- [x] T004 Create tests/unit/__init__.py and tests/integration/__init__.py directories

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create configuration loading module in agent.py (load env vars, set defaults)
- [x] T006 [P] Implement Pydantic models for QueryRequest in agent.py
- [x] T007 [P] Implement Pydantic models for QueryResponse in agent.py
- [x] T008 [P] Implement Pydantic models for SourceCitation in agent.py
- [x] T009 [P] Implement Pydantic models for ErrorResponse in agent.py
- [x] T010 Implement Qdrant client initialization and connection test function in agent.py
- [x] T011 Implement OpenAI client initialization in agent.py
- [x] T012 Create exception classes for RetrievalError, AgentError, ValidationError in agent.py
- [x] T013 Implement environment variable validation with dotenv in agent.py

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Ask Questions About the Full Book (Priority: P1) 🎯 MVP

**Goal**: Allow readers to submit natural language questions about the entire book and receive accurate, grounded answers from the RAG agent.

**Independent Test**: Submit a question via POST /query and verify response contains answer + sources with citations.

### Implementation for User Story 1

- [x] T014 [P] [US1] Implement retrieve_context function to query Qdrant in agent.py
- [x] T015 [P] [US1] Implement format_chunks_for_context function in agent.py
- [x] T016 [US1] Initialize OpenAI Agent with retrieval tool in agent.py (depends on T014)
- [x] T017 [US1] Implement POST /query endpoint for full-book queries in agent.py
- [x] T018 [US1] Implement extract_sources_from_result function in agent.py
- [x] T019 [US1] Add validation for question length (1-1000 chars) in agent.py
- [x] T020 [US1] Add error handling for empty results (NO_RELEVANT_CONTENT) in agent.py
- [x] T021 [US1] Add latency tracking for query responses in agent.py

**Checkpoint**: User Story 1 complete - full book Q&A should work independently

---

## Phase 4: User Story 2 - Ask Questions About Selected Text (Priority: P2)

**Goal**: Allow readers to scope questions to specific passages or selected text sections.

**Independent Test**: Submit a query with scope parameter and verify retrieval is limited to that scope.

### Implementation for User Story 2

- [ ] T022 [P] [US2] Implement parse_scope_parameter function in agent.py
- [ ] T023 [P] [US2] Implement scoped_retrieve_context function for passage-filtered queries in agent.py
- [ ] T024 [US2] Update retrieve_context to accept scope parameter in agent.py
- [ ] T025 [US2] Update POST /query endpoint to accept optional scope parameter in agent.py
- [ ] T026 [US2] Add validation for scope parameter values (full_book or passage:{id}) in agent.py
- [ ] T027 [US2] Handle SCOPE_NOT_FOUND error for invalid passage IDs in agent.py

**Checkpoint**: User Story 2 complete - selected text scoping should work

---

## Phase 5: User Story 3 - Validate Response Grounding (Priority: P2)

**Goal**: Ensure all responses include citations to source chunks, enabling readers to verify information.

**Independent Test**: Verify every response includes non-empty sources array with valid chunk references.

### Implementation for User Story 3

- [ ] T028 [P] [US3] Implement format_source_citation function in agent.py
- [ ] T029 [P] [US3] Implement extract_citations_from_context function in agent.py
- [ ] T030 [US3] Update agent instructions to include citation requirements in agent.py
- [ ] T031 [US3] Add citation enforcement (verify sources exist before returning) in agent.py
- [ ] T032 [US3] Add text_preview field to SourceCitation (first 100 chars) in agent.py
- [ ] T033 [US3] Update QueryResponse to always include sources in agent.py

**Checkpoint**: User Story 3 complete - all responses should include citations

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T034 [P] Implement GET /health endpoint for monitoring in agent.py
- [x] T035 [P] Implement GET /ready endpoint for deployment readiness in agent.py
- [x] T036 [P] Add structured logging to all endpoint handlers in agent.py
- [x] T037 [P] Add request ID tracking for tracing in agent.py
- [ ] T038 Implement 30-second timeout handling for queries in agent.py
- [ ] T039 Add RATE_LIMITED error handling (HTTP 429) in agent.py
- [x] T040 [P] Update .env.example with all required environment variables documented
- [ ] T041 Run quickstart validation - verify API matches documented contracts

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational - Can run in parallel with US1
- **User Story 3 (P2)**: Can start after Foundational - Can run in parallel with US1/US2

### Parallel Opportunities

- Phase 1: T002, T003, T004 can run in parallel
- Phase 2: T006, T007, T008, T009 can run in parallel
- Phase 3: T014, T015 can run in parallel
- All user stories can proceed in parallel after foundational phase

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test full book Q&A independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add Polish → Finalize
