# Implementation Plan: User Authentication & Content Personalization

**Branch**: `003-user-auth-personalization` | **Date**: 2026-02-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-user-auth-personalization/spec.md`

## Summary

Implement user authentication (signup, signin, password reset, account deletion) using Better-Auth with a Hono API backend and Neon Serverless Postgres. Build background assessment, chapter progress tracking, content depth personalization, and rule-based learning path recommendations. The frontend integrates into the Docusaurus site via theme swizzling and custom React pages, using Better-Auth's React client for session management.

## Technical Context

**Language/Version**: TypeScript 5.5+ (Node.js 20+)
**Primary Dependencies**: Better-Auth 1.3+, Hono 4.x, Drizzle ORM 0.40+, Resend 4.x
**Storage**: Neon Serverless Postgres (Drizzle adapter) — `@neondatabase/serverless`
**Testing**: Vitest (unit/integration), Playwright (e2e)
**Target Platform**: Serverless deployment (Vercel/Railway/Render) + Docusaurus SSG on GitHub Pages
**Project Type**: Web application (Docusaurus frontend + Hono API backend)
**Performance Goals**: Auth endpoints <200ms p95, page load overhead <200ms, personalization visible <1s
**Constraints**: Free-tier databases (Neon 0.5GB), free-tier email (Resend 100/day), <500ms total auth overhead
**Scale/Scope**: ~1,000 concurrent users, ~10,000 registered users, 12-18 chapters tracked

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| # | Principle | Status | Justification |
|---|-----------|--------|---------------|
| 1 | Production-Ready Code Quality | PASS | TypeScript with strict mode, Drizzle type-safe queries, Vitest testing |
| 2 | Pedagogical Effectiveness | PASS | Background assessment drives personalized learning paths; progress tracking enables scaffolded progression |
| 3 | Documentation Excellence | PASS | API contracts defined in OpenAPI; quickstart guide provided; data model documented |
| 4 | Accessibility & Inclusivity | PASS | Auth forms follow WCAG; assessment form uses clear labels; responsive design maintained |
| 5 | Practical Hands-On Learning | N/A | Auth feature is infrastructure, not chapter content. Does not introduce or remove exercises |
| 6 | Technical Accuracy & Currency | PASS | Better-Auth (current), Hono (current), Drizzle (current), Neon (current) — all actively maintained 2025-2026 |
| 7 | Personalization & Adaptability | PASS | Core feature: background assessment → computed level → content depth toggling → learning path. Directly implements this principle |
| 8 | Security & Best Practices | PASS | Better-Auth bcrypt hashing, HTTP-only cookies, session-based auth, .env for secrets, CORS config, rate limiting planned |

**Pre-design gate**: PASS — all relevant principles satisfied or not applicable.

### Post-Design Re-Check

| # | Principle | Status | Notes |
|---|-----------|--------|-------|
| 1 | Production-Ready Code Quality | PASS | Drizzle schema with proper constraints, validation on all API inputs |
| 7 | Personalization & Adaptability | PASS | Assessment → computedLevel → `<PersonalizedDetails>` component → learning path engine |
| 8 | Security & Best Practices | PASS | No plain-text passwords, session tokens in HTTP-only cookies, soft-delete for data retention compliance |

**Post-design gate**: PASS — no violations introduced during design.

## Project Structure

### Documentation (this feature)

```text
specs/003-user-auth-personalization/
├── plan.md              # This file
├── research.md          # Phase 0: Technology research and decisions
├── data-model.md        # Phase 1: Database schema and entity definitions
├── quickstart.md        # Phase 1: Setup and development guide
├── contracts/
│   └── auth-api.yaml    # Phase 1: OpenAPI 3.1 contract for all endpoints
└── tasks.md             # Phase 2: Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── db/
│   │   ├── schema.ts            # Drizzle schema (all tables)
│   │   ├── index.ts             # Database connection (Neon)
│   │   └── migrations/          # Drizzle migration files
│   ├── auth/
│   │   └── index.ts             # Better-Auth instance configuration
│   ├── routes/
│   │   ├── assessment.ts        # Background assessment CRUD
│   │   ├── progress.ts          # Chapter progress tracking
│   │   ├── learning-path.ts     # Learning path generation
│   │   └── profile.ts           # User profile management
│   ├── services/
│   │   ├── assessment.service.ts    # Assessment business logic + level computation
│   │   ├── progress.service.ts      # Progress aggregation + "continue" logic
│   │   ├── learning-path.service.ts # Rule-based path generation engine
│   │   └── email.service.ts         # Resend email wrapper
│   ├── middleware/
│   │   └── auth.ts              # Session validation middleware for custom routes
│   └── index.ts                 # Hono app entry point + Better-Auth handler mount
├── tests/
│   ├── unit/
│   │   ├── assessment.service.test.ts
│   │   ├── progress.service.test.ts
│   │   └── learning-path.service.test.ts
│   └── integration/
│       ├── auth.test.ts
│       ├── assessment.test.ts
│       └── progress.test.ts
├── drizzle.config.ts            # Drizzle Kit configuration
├── package.json
├── tsconfig.json
└── .env.example

frontend/
├── src/
│   ├── lib/
│   │   └── auth-client.ts       # Better-Auth React client instance
│   ├── hooks/
│   │   └── useAuth.ts           # Custom auth hook wrapping Better-Auth
│   ├── components/
│   │   ├── AuthProvider.tsx      # React Context provider for auth state
│   │   ├── AssessmentForm.tsx    # Background assessment form component
│   │   ├── ProgressBar.tsx       # Module/chapter progress visualization
│   │   ├── LearningPathView.tsx  # Learning path display component
│   │   ├── PersonalizedDetails.tsx  # Content depth toggle component
│   │   └── ProtectedRoute.tsx    # Auth-required route wrapper
│   ├── pages/
│   │   ├── auth/
│   │   │   ├── signin.tsx        # Sign in page
│   │   │   ├── signup.tsx        # Sign up page
│   │   │   ├── forgot-password.tsx
│   │   │   └── reset-password.tsx
│   │   ├── dashboard.tsx         # Progress dashboard
│   │   ├── profile.tsx           # User profile + assessment edit
│   │   └── assessment.tsx        # Standalone assessment page
│   └── theme/
│       ├── Root.tsx              # Swizzled: wraps site with AuthProvider
│       ├── Navbar/               # Swizzled: adds auth buttons to navbar
│       │   └── Content/
│       │       └── index.tsx
│       └── DocItem/              # Swizzled: adds personalization to doc pages
│           └── Content/
│               └── index.tsx
├── package.json
└── docusaurus.config.ts
```

**Structure Decision**: Web application with separated `backend/` (Hono + Better-Auth API) and `frontend/` (Docusaurus + React). This separation allows:
- Independent deployment (API on Vercel/Railway, static site on GitHub Pages)
- Clear ownership boundary between auth/API logic and content rendering
- Feature 002 (RAG chatbot) can be a separate Python service sharing the same database
- Both projects share types via a common schema definition

## Complexity Tracking

> No constitution violations detected. All design decisions align with principles.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| — | — | — |
