---
id: 006
title: Plan User Auth and Personalization
stage: plan
date: 2026-02-10
surface: agent
model: claude-opus-4-6
feature: user-auth-personalization
branch: 003-user-auth-personalization
user: architect
command: /sp.plan
labels: ["authentication", "personalization", "better-auth", "architecture", "planning"]
links:
  spec: specs/003-user-auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-user-auth-personalization/plan.md
  - specs/003-user-auth-personalization/research.md
  - specs/003-user-auth-personalization/data-model.md
  - specs/003-user-auth-personalization/quickstart.md
  - specs/003-user-auth-personalization/contracts/auth-api.yaml
tests:
  - none (planning phase - no code written)
---

## Prompt

/sp.plan

(Triggered from feature branch 003-user-auth-personalization with existing spec.md covering user authentication, background assessment, progress tracking, content depth personalization, and learning path recommendations.)

## Response snapshot

Executed full Phase 0 + Phase 1 planning for the User Authentication & Content Personalization feature:

**Phase 0 — Research (research.md):**
- Selected Better-Auth 1.3+ as auth framework (constitution-mandated) with email/password, password reset, account deletion, session management
- Selected Hono as lightweight Node.js API framework for Better-Auth handler + custom routes
- Selected Neon Serverless Postgres with Drizzle ORM (constitution-mandated)
- Selected Resend for transactional email (password reset)
- Selected Vitest for testing, Playwright for e2e
- Documented Docusaurus integration pattern: Root.tsx wrapping with AuthProvider, theme swizzling for Navbar and DocItem
- Documented content personalization via client-side `<details>` element toggling
- Documented rule-based learning path engine mapping 5 goals to 4 modules

**Phase 1 — Design (data-model.md, contracts/, quickstart.md):**
- Defined 7 database tables: user, session, account, verification (Better-Auth managed) + background_assessment, chapter_progress, learning_path (custom)
- Created OpenAPI 3.1 contract with 14 endpoints across auth, assessment, progress, learning-path, and profile
- Defined computed level algorithm from assessment responses
- Documented cascade rules, indexes, and data retention policy (30-day soft-delete purge)
- Created quickstart guide with setup steps, environment variables, and key endpoints

**Constitution Check:** All 8 principles evaluated. All PASS or N/A. No violations. No complexity tracking needed.

**Plan.md:** Filled with technical context, constitution checks (pre and post-design), source code structure (backend/ + frontend/), and structure decision rationale.

## Outcome

- ✅ Impact: Complete implementation plan ready for /sp.tasks — 5 artifacts generated
- 🧪 Tests: N/A (planning phase)
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, quickstart.md, contracts/auth-api.yaml)
- 🔁 Next prompts: /sp.tasks to generate implementation tasks, then /sp.implement
- 🧠 Reflection: Better-Auth + Hono + Drizzle stack is clean and well-documented. The separation of Better-Auth managed tables vs custom tables is important for avoiding schema conflicts. Content personalization via client-side details toggling keeps Docusaurus SSG benefits intact.

## Evaluation notes (flywheel)

- Failure modes observed: None — spec was comprehensive, constitution had clear tech stack guidance
- Graders run and results (PASS/FAIL): Constitution Check PASS (all 8 principles), Post-Design Re-Check PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run /sp.tasks to validate that plan artifacts generate clean, implementable tasks
