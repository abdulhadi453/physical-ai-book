# Tasks: RAG Chatbot Frontend Integration

**Input**: Design documents from `/specs/005-rag-frontend-integration/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/api-contract.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create frontend directory structure for Chatbot component at physical-ai-book/src/components/Chatbot/
- [x] T002 Create frontend services directory at physical-ai-book/src/services/
- [x] T003 [P] Install additional npm dependencies if needed (none required - React/TypeScript already available)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Refactor backend/agent.py to create backend/api.py with FastAPI endpoints, CORS, error handlers
- [x] T005 Keep RAG agent logic in backend/agent.py (Config, QdrantService, RAGAgent, embedding functions)
- [x] T006 [P] Create TypeScript type definitions in physical-ai-book/src/components/Chatbot/types.ts for ChatMessage, SourceCitation
- [x] T007 [P] Implement API service functions in physical-ai-book/src/services/chatbotApi.ts (queryBook, checkHealth)
- [x] T008 Add CORS middleware to backend/api.py allowing requests from http://localhost:3000

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Submit Questions to Chatbot (Priority: P1) 🎯 MVP

**Goal**: Allow readers to submit natural language questions about the entire book and receive accurate, grounded answers from the RAG agent.

**Independent Test**: Submit a question via chatbot UI and verify response contains answer + sources with citations.

### Implementation for User Story 1

- [x] T009 [P] [US1] Create ChatInput component in physical-ai-book/src/components/Chatbot/ChatInput.tsx with input field and submit button
- [x] T010 [P] [US1] Create ChatMessage component in physical-ai-book/src/components/Chatbot/ChatMessage.tsx for displaying individual messages
- [x] T011 [P] [US1] Create SourceCitation component in physical-ai-book/src/components/Chatbot/SourceCitation.tsx for displaying citations
- [x] T012 [P] [US1] Create LoadingIndicator component in physical-ai-book/src/components/Chatbot/LoadingIndicator.tsx
- [x] T013 [US1] Implement main Chatbot component in physical-ai-book/src/components/Chatbot/index.tsx with state management (messages, isLoading, error, inputValue)
- [x] T014 [US1] Add handleSubmit function to send question to backend via chatbotApi.queryBook()
- [x] T015 [US1] Add input validation (non-empty, max 1000 chars) before API call
- [x] T016 [US1] Display loading indicator while waiting for backend response (isLoading state)
- [x] T017 [US1] Handle successful response - add assistant message with answer and sources to messages array
- [x] T018 [US1] Handle error responses - display user-friendly error message in UI
- [x] T019 [US1] Add styling for chatbot components in physical-ai-book/src/components/Chatbot/styles.css
- [x] T020 [US1] Integrate Chatbot component into a documentation page (e.g., physical-ai-book/docs/intro.md)

**Checkpoint**: User Story 1 complete - full book Q&A should work independently

---

## Phase 4: User Story 2 - View Chat History (Priority: P2)

**Goal**: Allow readers to review previous questions and answers without re-entering them.

**Independent Test**: Submit multiple questions and verify all previous question-answer pairs remain visible and scrollable.

### Implementation for User Story 2

- [x] T021 [US2] Implement scrollable message container in Chatbot component to display all messages in messages array
- [x] T022 [US2] Add auto-scroll to bottom when new message is added to conversation history
- [x] T023 [US2] Style message list for proper scrolling (max-height, overflow-y: auto) in styles.css
- [x] T024 [US2] Ensure messages array maintains order (user question, then assistant response)
- [x] T025 [US2] Add visual distinction between user messages and assistant messages (different styling/alignment)

**Checkpoint**: User Story 2 complete - chat history should display and scroll properly

---

## Phase 5: User Story 3 - Clear Chat Interface (Priority: P3)

**Goal**: Allow users to start a new conversation by clearing the current chat history.

**Independent Test**: Click clear button and verify chat interface is empty and ready for new input.

### Implementation for User Story 3

- [x] T026 [US3] Add "Clear Chat" button to Chatbot component UI
- [x] T027 [US3] Implement handleClear function to reset messages array to empty
- [x] T028 [US3] Clear inputValue state when clearing chat history
- [x] T029 [US3] Clear error state when clearing chat history
- [x] T030 [US3] Style clear button appropriately (position, color, hover state)

**Checkpoint**: User Story 3 complete - clear functionality should work

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T031 [P] Add error boundary to Chatbot component to prevent crashes (FR-012) - OPTIONAL
- [x] T032 [P] Implement network timeout handling in chatbotApi.ts
- [x] T033 [P] Add accessibility attributes (ARIA labels, keyboard navigation) to chatbot components
- [x] T034 [P] Test responsive design for mobile/tablet screen sizes
- [x] T035 [P] Add loading state timeout (show error if response takes >30 seconds)
- [x] T036 Run quickstart validation - verify end-to-end scenarios from quickstart.md work correctly
- [x] T037 Verify all acceptance criteria from spec.md are met (SC-001 through SC-006)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Can run in parallel with US1
  - User Story 3 (P3): Can start after Foundational - Can run in parallel with US1/US2
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 (uses messages array) but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Builds on US1 (modifies messages state) but independently testable

### Parallel Opportunities

- Phase 1: T002, T003 can run in parallel
- Phase 2: T006, T007 can run in parallel (different files)
- Phase 3: T009, T010, T011, T012 can run in parallel (different component files)
- All user stories can proceed in parallel after foundational phase (if team capacity allows)

---

## Parallel Example: User Story 1

```bash
# Launch all component files for User Story 1 together:
Task: "Create ChatInput component in physical-ai-book/src/components/Chatbot/ChatInput.tsx"
Task: "Create ChatMessage component in physical-ai-book/src/components/Chatbot/ChatMessage.tsx"
Task: "Create SourceCitation component in physical-ai-book/src/components/Chatbot/SourceCitation.tsx"
Task: "Create LoadingIndicator component in physical-ai-book/src/components/Chatbot/LoadingIndicator.tsx"

# These can all be worked on at the same time by different developers or agents
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently - submit test question, verify response displays
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add Polish → Finalize

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (core Q&A)
   - Developer B: User Story 2 (history display) in parallel
   - Developer C: User Story 3 (clear functionality) in parallel
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Backend refactor (T004-T005) is critical - all frontend work depends on it
- No tests generated - spec explicitly uses manual end-to-end testing strategy
- All file paths use absolute references based on project structure in plan.md
