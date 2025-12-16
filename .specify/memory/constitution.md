<!-- Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: Updated MCP Infrastructure Compliance, Context7 Grounding, Technology Stack Requirements
Added sections: Scope Boundaries, Style Guidelines, Cover Text
Removed sections: None
Templates requiring updates: ✅ updated / ⚠ pending
- .specify/templates/plan-template.md (constitution check section)
- .specify/templates/spec-template.md (requirements alignment)
- .specify/templates/tasks-template.md (task categorization)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### Physical AI Focus
AI systems operating under real-world physical laws; All content must demonstrate how AI algorithms behave in physical environments; Clear connection between digital intelligence and real-world physics

### Embodied Intelligence
Digital brain + physical body integration; Content must cover the complete loop from sensors → AI → actuators; Focus on how intelligence emerges through physical interaction

### Spec-Driven Content Generation
All content generation follows spec-driven methodology; Content must be reproducible and testable; Clear purpose required - no content without clear learning outcomes

### MCP Infrastructure Compliance
Context7 MCP Server as primary documentation source; All agents (Writer, Explainer, Chatbot, Quiz Agent) must query Context7 MCP for: ROS 2 APIs, Gazebo documentation, Unity robotics and Digital Twin integration, NVIDIA Isaac Sim, Isaac ROS, Omniverse APIs, Docusaurus documentation; Tool calls MUST occur before content generation; MCP responses must be summarized, never copied verbatim; No hallucinated APIs, flags, parameters, or workflows allowed

### Context7 Grounding
All technical content grounded in Context7 MCP retrieved documentation; If Context7 MCP does not contain information, respond with "Not covered in this book"; Every explanation must be grounded in Context7 MCP documentation; No agent may rely on internal memory alone

### Beginner-Friendly Rigor
Content must be technically rigorous but accessible to beginners; Explanation flow: Concept → Example → Physical-world application; Zero hallucination tolerance in RAG chatbot

## Scope Boundaries

Allowed topics ONLY:
- Physical AI
- Humanoid Robotics
- ROS 2
- Gazebo Simulation
- Unity Digital Twins
- NVIDIA Isaac Sim & Isaac ROS
- Navigation, SLAM, Nav2
- Vision-Language-Action graphs (Gazebo / Isaac simulation grid)
- Human–robot interaction feel

Prohibited topics:
- Any content outside the specified scope boundaries
- Hallucinated APIs, flags, parameters, or workflows

## Style Guidelines

- Futuristic + Educational aesthetic
- Dark background (black / navy blue)
- Neon blue / purple AI lines
- All content authored in Markdown
- Rendered automatically via Docusaurus
- Estimated pages: 200–400 total, 15–40 pages per chapter

## Technology Stack Requirements

Technology stack requirements: Python & ROS 2 for robotics content, Docusaurus for documentation website, OpenAI Agents or ChatKit SDK for RAG chatbot, FastAPI backend, Neon Serverless Postgres, Qdrant Cloud for vector database
Compliance standards: Technical accuracy, MCP grounding requirements, no out-of-scope content
Deployment policies: Context7 MCP used for all technical grounding, exactly 12 chapters, mobile-responsive layout

## Development Workflow

Code review requirements: All PRs must verify compliance with spec requirements and Context7 MCP grounding; Testing gates: All chapters must pass content accuracy checks before merge; Content approval process: Technical accuracy review required for all robotics/AI content

## Cover Text

**Title:** Physical AI & Humanoid Robotics
**Subtitle:** From Digital Intelligence to Embodied Systems
**Footer:** Panaversity – AI Native Education

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan; All PRs/reviews must verify compliance; Complexity must be justified; Use `.specify/memory/constitution.md` for runtime development guidance; Content must be 100% compliant with scope boundaries; Context7 MCP = single source of technical truth

**Version**: 1.1.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
