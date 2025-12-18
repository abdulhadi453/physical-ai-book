---
id: 0001
title: Physical AI Book Constitution Creation
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-book
branch: 001-physical-ai-book
user: ICTech
command: /sp.constitution you have already a file of constitution update a file according to this.

You are a Principal Software Engineer, AI Architect, and Robotics Curriculum Designer with 20+ years of experience.

You strictly follow Spec-Driven Development.

TASK:
Create the sp.constitution for the following project.

The sp.constitution is the highest governing specification.
All future specs, content, code, agents, and architecture MUST comply with it.
Any violation is grounds for rejection.

DO NOT write content, chapters, or outlines.
DO NOT ask questions.

PROJECT:
Title: Physical AI & Humanoid Robotics
Type: AI-Native Technical Textbook + Interactive Learning System
Context: Panaversity Hackathon project evaluated by senior AI engineers and educators.

OBJECTIVE:
Build an AI-native textbook (Docusaurus + GitHub Pages) that teaches Physical AI and Humanoid Robotics using ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems, with embedded AI agents and RAG-based learning.

MANDATORY REQUIREMENTS:
- Spec-Driven Development is REQUIRED (no content without approved specs)
- Claude Code and Spec-Kit Plus MUST be used
- Content MUST be chunked for RAG
- Embedded chatbot MUST answer strictly from book content
- "Answer from selected text only" mode is REQUIRED
- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant
- Reusable intelligence via Claude subagents is REQUIRED
- Authentication via better-auth is REQUIRED
- User profiling at signup (software + hardware background) is REQUIRED
- Chapter-level personalization button is REQUIRED
- Chapter-level Urdu translation button is REQUIRED

ACADEMIC & PEDAGOGY:
- Audience: CS / Engineering background
- Theory → Simulation → Deployment progression
- Robotics claims MUST respect physical constraints
- Simulation vs real-world limits MUST be explicit
- Safety and ethics MUST be addressed

OUTPUT FORMAT (EXACT):
1. Project Definition
2. Purpose of the Constitution
3. Core Principles
4. Spec-Driven Development Rules
5. AI-Native Design Mandates
6. Content & Pedagogy Standards
7. Technical Stack Constraints
8. Verification & Reproducibility Rules
9. Ethical & Safety Requirements
10. Evaluation Alignment
11. Success Criteria
12. Non-Compliance Policy

LANGUAGE RULES:
- Formal, enforcement-oriented
- Use ONLY: MUST, MUST NOT, REQUIRED, FORBIDDEN
- No marketing language
- Every rule must be testable
labels: ["constitution","update","physical-ai","robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests:[]
---

## Prompt

you have already a file of constitution update a file according to this.

You are a Principal Software Engineer, AI Architect, and Robotics Curriculum Designer with 20+ years of experience.

You strictly follow Spec-Driven Development.

TASK:
Create the sp.constitution for the following project.

The sp.constitution is the highest governing specification.
All future specs, content, code, agents, and architecture MUST comply with it.
Any violation is grounds for rejection.

DO NOT write content, chapters, or outlines.
DO NOT ask questions.

PROJECT:
Title: Physical AI & Humanoid Robotics
Type: AI-Native Technical Textbook + Interactive Learning System
Context: Panaversity Hackathon project evaluated by senior AI engineers and educators.

OBJECTIVE:
Build an AI-native textbook (Docusaurus + GitHub Pages) that teaches Physical AI and Humanoid Robotics using ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems, with embedded AI agents and RAG-based learning.

MANDATORY REQUIREMENTS:
- Spec-Driven Development is REQUIRED (no content without approved specs)
- Claude Code and Spec-Kit Plus MUST be used
- Content MUST be chunked for RAG
- Embedded chatbot MUST answer strictly from book content
- "Answer from selected text only" mode is REQUIRED
- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant
- Reusable intelligence via Claude subagents is REQUIRED
- Authentication via better-auth is REQUIRED
- User profiling at signup (software + hardware background) is REQUIRED
- Chapter-level personalization button is REQUIRED
- Chapter-level Urdu translation button is REQUIRED

ACADEMIC & PEDAGOGY:
- Audience: CS / Engineering background
- Theory → Simulation → Deployment progression
- Robotics claims MUST respect physical constraints
- Simulation vs real-world limits MUST be explicit
- Safety and ethics MUST be addressed

OUTPUT FORMAT (EXACT):
1. Project Definition
2. Purpose of the Constitution
3. Core Principles
4. Spec-Driven Development Rules
5. AI-Native Design Mandates
6. Content & Pedagogy Standards
7. Technical Stack Constraints
8. Verification & Reproducibility Rules
9. Ethical & Safety Requirements
10. Evaluation Alignment
11. Success Criteria
12. Non-Compliance Policy

LANGUAGE RULES:
- Formal, enforcement-oriented
- Use ONLY: MUST, MUST NOT, REQUIRED, FORBIDDEN
- No marketing language
- Every rule must be testable

## Response snapshot

Constitution file updated to follow the 12-section format with formal, enforcement-oriented language using mandatory requirements. The new constitution establishes:

1. Project Definition: Physical AI & Humanoid Robotics as AI-Native Technical Textbook
2. Purpose: Highest governing specification with constitutional compliance requirements
3. Core Principles: Spec-Driven Dev, AI-Native Integration, Technical Rigor, Accessibility, Safety
4. Spec-Driven Development Rules: Mandatory spec compliance requirements
5. AI-Native Design Mandates: RAG chunking, chatbot constraints, subagent requirements
6. Content & Pedagogy Standards: Audience-focused with theory→simulation→deployment
7. Technical Stack Constraints: Specific tech stack requirements
8. Verification & Reproducibility: Testing and validation requirements
9. Ethical & Safety Requirements: Safety protocols and ethical considerations
10. Evaluation Alignment: Evaluation criteria by senior engineers/educators
11. Success Criteria: Student outcome and system capability requirements
12. Non-Compliance Policy: Enforcement procedures for violations

The constitution was completely rewritten from version 1.2.0 to 2.0.0 with formal, enforcement-oriented language using mandatory requirements (MUST, REQUIRED, FORBIDDEN) as specified.

## Outcome

- ✅ Impact: Major update to constitution with formal enforcement language
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new 12-section structure
- 🔁 Next prompts: Continue with spec development following constitutional requirements
- 🧠 Reflection: Constitution now properly structured with mandatory requirements using enforcement language

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin spec development aligned with new constitution
