# Implementation Plan: Agent-Based RAG Backend with FastAPI

**Branch**: `004-agent-rag-api` | **Date**: 2025-12-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/sp.specify` command

## Summary

Build an intelligent RAG agent that answers questions about the Physical AI book using stored embeddings from Qdrant. The system uses OpenAI Agents SDK for reasoning and response generation, exposed via a FastAPI backend. Users can query the full book or scope questions to selected text passages. All responses must be grounded in retrieved context with source citations.

## Technical Context

**Language/Version**: Python 3.10+ |
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant client |
**Storage**: Qdrant Cloud (vector database), no local persistence |
**Testing**: pytest (unit + integration) |
**Target Platform**: Linux server (containerized) |
**Project Type**: Single Python file (`agent.py`) |
**Performance Goals**: Response time <30s for 95% of queries |
**Constraints**: Single-file implementation, grounded responses only, no external knowledge |
**Scale/Scope**: Single-user reads from book content, supports sequential queries |

## Constitution Check

*GATE: Must pass before implementation.*

- No constitutional violations detected - feature follows single-file simplicity principle
- Test-first approach will be followed during implementation
- Integration testing will cover Qdrant retrieval and OpenAI API interactions

## Project Structure

### Documentation (this feature)

```text
specs/004-agent-rag-api/
├── plan.md              # This file
├── spec.md              # Feature specification
├── tasks.md             # Implementation tasks
└── checklists/          # Quality checklists
    └── requirements.md
```

### Source Code (repository root)

```text
agent.py                 # Single file RAG agent with FastAPI endpoints
.env.example             # Environment variables template
requirements.txt         # Python dependencies
tests/                   # Test directory
├── __init__.py
├── unit/
│   └── __init__.py
└── integration/
    └── __init__.py
```

**Structure Decision**: Single-file Python implementation (`agent.py`) at project root. All code lives in one file for simplicity as specified.
