---
id: 003
title: Create Chapter Author Skill
stage: general
date: 2026-02-10
surface: agent
model: claude-opus-4-6
feature: none
branch: 001-docusaurus-textbook-site
user: architect
command: /skills:skill-creator
labels: ["skill", "chapter-author", "constitution", "content-generation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/chapter-author/SKILL.md
  - .claude/commands/chapter-author/references/modules.md
  - .claude/commands/chapter-author/references/constitution-checklist.md
  - .claude/commands/chapter-author/assets/chapter-template.mdx
  - dist/chapter-author.skill
tests:
  - quick_validate.py: PASS (Skill is valid!)
  - package_skill.py: PASS (packaged to dist/chapter-author.skill)
---

## Prompt

constitution ke requriremtns ke mutabiq skill-creator ke through skill create kro jo is project mein kaam aye

(Translation: Create a skill using skill-creator following the constitution's requirements that will be useful for this project)

## Response snapshot

Created `chapter-author` skill using the skill-creator workflow. The skill generates constitution-compliant chapter MDX files for the Physical AI & Humanoid Robotics Docusaurus textbook. Includes: SKILL.md with complete workflow (code example rules, exercise format, accessibility patterns, Urdu scaffolding, security patterns, personalization markers), references/modules.md (4 modules + special chapters detail), references/constitution-checklist.md (all 8 principles as validation checklist), assets/chapter-template.mdx (full MDX template with Tabs, admonitions, collapsible sections). Validated and packaged to dist/chapter-author.skill.

## Outcome

- Impact: New skill enables consistent, constitution-compliant chapter authoring
- Tests: Skill validation PASS, packaging PASS
- Files: 4 skill files created + 1 .skill package
- Next prompts: Use `chapter-author` skill to generate first chapter, or continue with /sp.plan
- Reflection: Constitution's detailed principle rules mapped well to skill guardrails

## Evaluation notes (flywheel)

- Failure modes observed: init_skill.py failed due to Windows cp1252 emoji encoding; used manual creation fallback
- Graders run and results (PASS/FAIL): quick_validate PASS, package_skill PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test skill by generating a Module 1 chapter
