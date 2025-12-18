<!-- Sync Impact Report:
Version change: 1.2.0 -> 2.0.0 (major rewrite to match user requirements)
Modified principles: Completely replaced with new structure following specified 12 sections
Added sections: All 12 required sections with formal, enforcement-oriented language
Removed sections: Previous informal structure
Templates requiring updates: .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Constitution

## 1. Project Definition

Physical AI & Humanoid Robotics: AI-Native Technical Textbook + Interactive Learning System. A panaversity hackathon project evaluated by senior AI engineers and educators. This project MUST deliver a Docusaurus-based textbook teaching Physical AI and Humanoid Robotics using ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action systems, with embedded AI agents and RAG-based learning.

## 2. Purpose of the Constitution

This constitution is the highest governing specification. All future specs, content, code, agents, and architecture MUST comply with it. Any violation is grounds for rejection. This document establishes mandatory requirements and non-negotiable constraints that govern all project development activities.

## 3. Core Principles

- **Spec-Driven Development**: All content, code, and architecture MUST originate from approved specifications. No implementation without approved specs is permitted.
- **AI-Native Integration**: All components MUST leverage AI agents and RAG systems for enhanced learning experiences.
- **Technical Rigor**: All robotics claims MUST respect physical constraints. Simulation vs real-world limits MUST be explicit.
- **Accessibility**: Content MUST be accessible to CS/Engineering audience with appropriate theory → simulation → deployment progression.
- **Safety First**: Ethics and safety MUST be addressed in all technical content.

## 4. Spec-Driven Development Rules

- Spec-Driven Development is REQUIRED: No content, code, or architecture development without approved specifications is permitted.
- Claude Code and Spec-Kit Plus MUST be used for all development activities.
- All specifications MUST pass constitutional compliance checks before approval.
- Any deviation from approved specs MUST trigger constitutional review.

## 5. AI-Native Design Mandates

- Content MUST be chunked for RAG to enable effective retrieval-augmented generation.
- Embedded chatbot MUST answer strictly from book content only.
- "Answer from selected text only" mode is REQUIRED to prevent hallucinations.
- Reusable intelligence via Claude subagents is REQUIRED for scalable learning assistance.
- AI agents MUST enhance education while preserving human comprehension.

## 6. Content & Pedagogy Standards

- Audience: CS/Engineering background with theory → simulation → deployment progression.
- Robotics claims MUST respect physical constraints and real-world limitations.
- Simulation vs real-world limits MUST be explicitly documented and explained.
- Safety and ethics MUST be addressed in all technical content.
- All content MUST support the pedagogical progression from theory to practice.

## 7. Technical Stack Constraints

- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant is REQUIRED.
- Authentication via better-auth is REQUIRED for all user interactions.
- User profiling at signup (software + hardware background) is REQUIRED.
- Chapter-level personalization button is REQUIRED for adaptive learning.
- Chapter-level Urdu translation button is REQUIRED for accessibility.
- Docusaurus + GitHub Pages is the mandated delivery platform.

## 8. Verification & Reproducibility Rules

- All technical examples MUST be reproducible in both simulation and real-world environments.
- All code examples MUST include verification procedures and expected outcomes.
- All simulations MUST include validation against physical constraints.
- All deployments MUST include verification of real-world safety requirements.
- Reproducibility tests MUST be included in all deliverables.

## 9. Ethical & Safety Requirements

- All robotic systems described MUST include safety protocols and risk assessments.
- All AI systems MUST include ethical considerations and bias mitigation strategies.
- All hardware interactions MUST include safety warnings and protective measures.
- All simulation-to-reality transfers MUST include safety validation procedures.
- All content MUST comply with ethical AI development practices.

## 10. Evaluation Alignment

- Content evaluation MUST be performed by senior AI engineers and educators.
- All technical claims MUST be validated against real-world physical constraints.
- All AI integration MUST be evaluated for educational effectiveness.
- All safety requirements MUST be verified by qualified reviewers.
- All accessibility features MUST be tested with target audience groups.

## 11. Success Criteria

- Students MUST be able to implement Physical AI concepts in simulation and deployment.
- Students MUST demonstrate understanding of simulation vs real-world limitations.
- Students MUST complete projects that respect physical constraints and safety requirements.
- Students MUST utilize AI assistance tools appropriately while maintaining comprehension.
- The system MUST support the specified technical stack and accessibility requirements.

## 12. Non-Compliance Policy

- Any deliverable that violates constitutional requirements MUST be rejected immediately.
- Any specification that conflicts with constitutional requirements MUST be revised.
- Any implementation that bypasses constitutional constraints MUST be discarded.
- Any content that fails to meet mandatory requirements MUST be redeveloped.
- Constitutional compliance is the responsibility of all contributors and reviewers.

**Version**: 2.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-15