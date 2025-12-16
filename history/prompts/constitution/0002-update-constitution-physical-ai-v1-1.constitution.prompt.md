---
id: 0002
title: Update-constitution-physical-ai-v1-1
stage: constitution
date: 2025-12-14
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: bdullah
command: /sp.constitution
labels: ["constitution", "spec-driven", "physical-ai", "humanoid-robotics", "v1.1-update"]
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

# 📘 Physical AI & Humanoid Robotics

## Full-Length Technical Book + RAG Chatbot + Book Content Dataset (2023–2033)

---

## 🎯 Core Principles

* **Physical AI focus**: AI systems operating under real-world physical laws
* **Embodied intelligence**: Integration of a digital brain with a physical robot body
* **Beginner-friendly yet technically rigorous**
* **Spec-driven, reproducible content generation**

---

## 🧠 Mandatory MCP Infrastructure (STRICT)

### Context7 MCP Server (REQUIRED)

Context7 MCP Server is the **single source of technical truth** and MUST be used as the primary:

* Documentation context provider
* Content grounding layer
* Tool-aware knowledge interface

### Mandatory MCP Usage

All agents (Writer, Explainer, Chatbot, Quiz Agent) **must query Context7 MCP** for:

* ROS 2 APIs
* Gazebo documentation
* Unity robotics and Digital Twin integration
* NVIDIA Isaac Sim, Isaac ROS, Omniverse APIs
* Docusaurus documentation

### MCP Rules (Non-Negotiable)

* No agent may rely on internal memory alone
* Every explanation must be grounded in Context7 MCP documentation
* Tool calls MUST occur **before** content generation
* MCP responses must be **summarized**, never copied verbatim
* **No hallucinated APIs, flags, parameters, or workflows**

❌ If Context7 MCP does not contain the requested information, respond exactly with:

> **"Not covered in this book"**

---

## 🚫 Scope Boundaries (Strict Enforcement)

### ✅ Allowed Topics ONLY

* Physical AI
* Humanoid Robotics
* ROS 2
* Gazebo Simulation
* Unity Digital Twins
* NVIDIA Isaac Sim & Isaac ROS
* Navigation, SLAM, Nav2
* Vision-Language-Action graphs
  * Gazebo / Isaac simulation grid
* Human–robot interaction feel

### Style

* Futuristic + Educational
* Dark background (black / navy blue)
* Neon blue / purple AI lines

### Cover Text

**Title:**
*Physical AI & Humanoid Robotics*

**Subtitle:**
*From Digital Intelligence to Embodied Systems*

**Footer:**
*Panaversity – AI Native Education*

---

## 📊 Estimated Pages

* **Total Pages:** 200–400
* **Each Chapter:** 15–40 pages

Includes:

* Theory
* Code examples
* Diagrams
* Exercises
* Capstone walkthrough

📚 All content authored in Markdown
📌 Rendered automatically via Docusaurus
```

## Response snapshot

Updated the project constitution at `.specify/memory/constitution.md` to version 1.1.0 with additional details for the Physical AI & Humanoid Robotics project. The updated constitution now includes:

- Enhanced MCP Infrastructure Compliance principle with specific agent types (Writer, Explainer, Chatbot, Quiz Agent) and detailed technology requirements (ROS 2, Gazebo, Unity, NVIDIA Isaac, etc.)
- Enhanced Context7 Grounding with stricter requirements (no agent may rely on internal memory alone)
- New Scope Boundaries section with allowed and prohibited topics
- New Style Guidelines section with futuristic + educational aesthetic requirements
- New Cover Text section with title, subtitle and footer details
- Governance section updated to include Context7 MCP as single source of technical truth

Version updated from 1.0.0 to 1.1.0 as this is a minor update with additional sections and enhanced requirements.

## Outcome

- ✅ Impact: Project constitution now includes detailed Physical AI & Humanoid Robotics requirements with specific scope boundaries and style guidelines
- 🧪 Tests: No tests required for constitution update
- 📁 Files: .specify/memory/constitution.md updated from v1.0.0 to v1.1.0 with additional sections
- 🔁 Next prompts: Ready to proceed with spec creation for the Physical AI book project based on updated constitution
- 🧠 Reflection: Constitution now comprehensively covers all project requirements including MCP infrastructure, scope boundaries, and style guidelines

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create feature specification based on updated constitution v1.1
