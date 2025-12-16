---
id: 0001
title: Update-constitution-physical-ai
stage: constitution
date: 2025-12-14
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: bdullah
command: /sp.constitution
labels: ["constitution", "spec-driven", "physical-ai", "humanoid-robotics"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

```text
    Project: Physical AI & Humanoid Robotics – Full-Length Technical Book + RAG Chatbot + Book contant Data (2023 sy 2033)

Core principles:

- Physical AI focus: AI systems operating under real-world physical laws
- Embodied intelligence (digital brain + physical body integration)
- Beginner-friendly but technically rigorous
- Spec-driven, reproducible content generation

────────────────────────────────
MANDATORY MCP INFRASTRUCTURE
────────────────────────────────

Context7 MCP Server (REQUIRED):
- Context7 MCP Server MUST be used as the primary:
  - Documentation context provider
  - Content grounding layer
  - Tool-aware knowledge interface
- All agents (writer, explainer, chatbot) must:
  - Query Context7 MCP for:
    - ROS 2 APIs
    - Gazebo documentation
    - Unity integration references
    - NVIDIA Isaac / Omniverse APIs
    - Docusaurus documentation
- No agent is allowed to rely on internal memory alone
- Every generated explanation must be grounded in:
  - Context7 MCP retrieved documentation
- If Context7 MCP does not contain the information:
  - The agent must respond with: "Not covered in this book"

MCP Usage Rules:
- Context7 MCP = single source of technical truth
- No hallucinated APIs, flags, or workflows
- Tool calls must precede content generation
- MCP responses must be summarized, not copied verbatim

────────────────────────────────
SCOPE BOUNDARIES (STRICT)
────────────────────────────────

Allowed topics ONLY:
- Physical AI
- Humanoid Robotics
- umentation website
  - Mobile-friendly layout
- Each chapter = one Docusaurus doc page
- Assets (images/diagrams) referenced via Docusaurus static paths

Docusaurus MCP Integration:
- Docusaurus documentation must be accessed via Context7 MCP
- Sidebar configuration must be spec-driven
- No custom docs framework allowed

────────────────────────────────
WRITING & CODE STANDARDS
────────────────────────────────

- Language: Technical but beginner-friendly
- Explanation flow: Concept → Example → Physical-world application
- Code:
  - Python & ROS 2 only
  - Educational, well-commented
  - Grounded in Context7 MCP references
- Diagrams:
  - Robot architecture
  - Data flow (sensors → AI → actuators)
  - Simulation vs real-world mapping

────────────────────────────────
LEARNING OUTCOMES
────────────────────────────────

Each chapter must clearly state:
- What the learner understands
- What the learner can build, simulate, or control
- Outcomes must reflect Physical AI skills

────────────────────────────────
RAG CHATBOT (CORE DELIVERABLE)
────────────────────────────────

Behavior rules:
- Answers ONLY from book content
- Chapter-aware responses
- Selected-text-aware responses
- Zero hallucination tolerance

Tech stack (MANDATORY):
- OpenAI Agents or ChatKit SDK
- FastAPI backend
- Neon Serverless Postgres (metadata + auth)
- Qdrant Cloud (vector DB – free tier)
- Context7 MCP as grounding layer

Failure behavior:
- If answer not found → "Not covered in this book"

────────────────────────────────
PLATFORM REQUIREMENTS
────────────────────────────────

- Docusaurus-based website
- Chapter-wise sidebar
- Search enabled
- Mobile responsive
- Embedded RAG chatbot UI

────────────────────────────────
BONUS (SPEC-COMPATIBLE)
────────────────────────────────

- Better-Auth for signup/signin
- User profiling (software vs robotics)
- Beginner / Advanced toggle
- English → Urdu translation toggle
- Sub-agents:
  - Chapter Writer Agent
  - Quiz Generator Agent
  - Code Explainer Agent

────────────────────────────────
DESIGN CONSTRAINTS
────────────────────────────────

- Book cover:
  - Physical AI theme
  - Humanoid robot + AI brain
  - Futuristic & professional
- UI:
  - Clean
  - Readable
  - Diagram-friendly

────────────────────────────────
SUCCESS CRITERIA
────────────────────────────────

- 100% compliance Book
- Context7 MCP used for all technical grounding
- Exactly 12 chapters
- No out-of-scope content
- Docusaurus renders cleanly
- RAG chatbot grounded & non-hallucinatory
- Fully spec-driven & reproducible
- 100% red Book and cexcells fullcexcells full person ⭐⭐⭐⭐⭐🤵👨‍🎓🏆🥇💰 2033
```

## Response snapshot

Updated the project constitution at `.specify/memory/constitution.md` to reflect the Physical AI & Humanoid Robotics project requirements. The new constitution includes:

- Core Principles: Physical AI Focus, Embodied Intelligence, Spec-Driven Content Generation, MCP Infrastructure Compliance, Context7 Grounding, Beginner-Friendly Rigor
- Technology Stack Requirements: Python & ROS 2, Docusaurus, OpenAI Agents/ChatKit SDK, FastAPI, Neon Postgres, Qdrant Cloud
- Development Workflow: MCP grounding requirements, technical accuracy reviews
- Governance: Compliance with scope boundaries and MCP infrastructure requirements

Version updated from 0.1.0 to 1.0.0 as this is a major update with new principles and project-specific requirements.

## Outcome

- ✅ Impact: Project constitution now accurately reflects Physical AI & Humanoid Robotics requirements
- 🧪 Tests: No tests required for constitution update
- 📁 Files: .specify/memory/constitution.md updated successfully
- 🔁 Next prompts: Ready to proceed with spec creation for the Physical AI book project
- 🧠 Reflection: Constitution now serves as the authoritative source for project governance and development practices

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specification based on updated constitution
