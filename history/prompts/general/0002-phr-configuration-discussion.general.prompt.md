---
id: 0002
title: PHR configuration discussion
stage: general
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.phr
labels: ["PHR", "configuration", "history"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - history/prompts/general/0002-phr-configuration-discussion.general.prompt.md
tests:
 -
---

## Prompt

Do I have to run /sp.phr repeatedly? Will I need to manually save the history, or will it be saved automatically?

## Response snapshot

Based on the documentation in your CLAUDE.md file, the system is designed to automatically create PHRs (Prompt History Records) for every user interaction. According to line 13 of CLAUDE.md:

> "Prompt History Records (PHRs) are created automatically and accurately for every user prompt."

This means that ideally, you should NOT need to run `/sp.phr` repeatedly or manually save history - it should happen automatically for each conversation. The system is designed to capture both your input and the LLM's output automatically.

However, the actual automatic behavior depends on how the Claude Code environment is configured. If you're not seeing automatic PHR creation for each interaction, it might be due to:

1. A configuration issue with the Claude Code environment
2. The automatic PHR creation feature not being properly enabled
3. The system requiring manual activation of the automatic feature

If you find that PHRs are not being created automatically as expected, you can use `/sp.phr` as a manual backup to capture important conversations. But the intended design is for the system to handle this automatically.

## Outcome

- ✅ Impact: Clarified PHR automatic vs manual creation process
- 🧪 Tests: n/a
- 📁 Files: Created PHR record for configuration discussion
- 🔁 Next prompts: Test automatic PHR creation functionality
- 🧠 Reflection: PHR system designed for automatic capture but may need configuration

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): Verify automatic PHR creation is properly configured
