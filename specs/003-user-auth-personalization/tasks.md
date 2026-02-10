# Tasks: User Authentication & Content Personalization

**Input**: Design documents from `/specs/003-user-auth-personalization/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/auth-api.yaml

**Tests**: Not explicitly requested in the feature specification. Test tasks are omitted. Add them later with TDD approach if desired.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/` (per plan.md structure decision)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and configuration for both backend and frontend

- [X] T001 Initialize backend Node.js project with package.json including better-auth, hono, @hono/node-server, drizzle-orm, @neondatabase/serverless, resend dependencies in backend/package.json
- [X] T002 [P] Configure TypeScript strict mode with Node.js 20+ target in backend/tsconfig.json
- [X] T003 [P] Create .env.example documenting DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, RESEND_API_KEY, FRONTEND_URL in backend/.env.example
- [X] T004 [P] Add better-auth client dependency to the Docusaurus frontend project in frontend/package.json

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Database, auth infrastructure, and shared frontend components that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T005 Define complete Drizzle schema for all 7 tables (user with additionalFields displayName/deletedAt, session, account, verification, background_assessment, chapter_progress, learning_path) with indexes, constraints, and cascade rules per data-model.md in backend/src/db/schema.ts
- [X] T006 [P] Create Neon Serverless Postgres connection module using @neondatabase/serverless driver in backend/src/db/index.ts
- [X] T007 [P] Create Drizzle Kit configuration pointing to schema.ts and Neon DATABASE_URL in backend/drizzle.config.ts
- [X] T008 Configure Better-Auth instance with drizzleAdapter, emailAndPassword enabled (minPasswordLength: 8), sendResetPassword callback using Resend, custom user additionalFields (displayName, deletedAt), and session configuration in backend/src/auth/index.ts
- [X] T009 [P] Implement Resend email service wrapper with sendPasswordReset method in backend/src/services/email.service.ts
- [X] T010 Create session validation middleware that extracts authenticated user from Better-Auth session cookie for custom Hono routes in backend/src/middleware/auth.ts
- [X] T011 Create Hono app entry point: mount Better-Auth handler on /api/auth/*, configure CORS for FRONTEND_URL, register custom route groups (assessment, progress, learning-path, profile) in backend/src/index.ts
- [X] T012 Create Better-Auth React client instance with baseURL pointing to backend in frontend/src/lib/auth-client.ts
- [X] T013 [P] Create AuthProvider React context component that wraps children with Better-Auth session state and exposes user/session/loading/assessment level via context in frontend/src/components/AuthProvider.tsx
- [X] T014 [P] Create useAuth custom hook providing signIn, signUp, signOut, session, user, isAuthenticated, and assessmentLevel from AuthProvider context in frontend/src/hooks/useAuth.ts
- [X] T015 Swizzle Docusaurus Root theme component to wrap entire site with AuthProvider in frontend/src/theme/Root.tsx
- [X] T016 [P] Create ProtectedRoute wrapper component that redirects unauthenticated users to /auth/signin while preserving the return URL in frontend/src/components/ProtectedRoute.tsx

**Checkpoint**: Foundation ready — Better-Auth serves /api/auth/* endpoints, Docusaurus has auth context available on every page. User story implementation can begin.

---

## Phase 3: User Story 1 — Sign Up and Sign In (Priority: P1) MVP

**Goal**: Students can create accounts, sign in, sign out, reset passwords, and see auth state in the navbar. The complete authentication lifecycle works end to end.

**Independent Test**: Create a new account with email/password/displayName, sign out, sign back in, verify user is recognized. Request password reset, use reset link, verify new password works. Delete account, verify session is terminated.

### Implementation for User Story 1

- [X] T017 [P] [US1] Create sign up page with email, password (with strength indicator), and display name form fields; call authClient.signUp.email on submit; redirect to /assessment on success in frontend/src/pages/auth/signup.tsx
- [X] T018 [P] [US1] Create sign in page with email/password form, "Remember Me" checkbox, and "Forgot Password?" link; call authClient.signIn.email on submit; redirect to /dashboard or callbackURL on success in frontend/src/pages/auth/signin.tsx
- [X] T019 [P] [US1] Create forgot password page with email input; call authClient.forgetPassword with redirectTo pointing to /auth/reset-password in frontend/src/pages/auth/forgot-password.tsx
- [X] T020 [P] [US1] Create reset password page that reads token from URL params; accepts new password and confirmation; calls authClient.resetPassword in frontend/src/pages/auth/reset-password.tsx
- [X] T021 [US1] Swizzle Navbar Content component to show "Sign In" / "Sign Up" buttons for anonymous users and a user menu dropdown (displayName, Profile, Dashboard, Sign Out) for authenticated users in frontend/src/theme/Navbar/Content/index.tsx
- [X] T022 [US1] Implement profile API routes: GET /api/profile returns user + assessment + progress summary + learning path; PATCH /api/profile updates displayName/name per contracts/auth-api.yaml in backend/src/routes/profile.ts
- [X] T023 [US1] Create profile page showing user info (displayName, email, createdAt), edit displayName form, and "Delete Account" button with password confirmation dialog in frontend/src/pages/profile.tsx
- [X] T024 [US1] Add client-side form validation to signup page: email format regex, password min 8 chars with at least 1 letter and 1 number (FR-002), display name required; show inline error messages before submission
- [X] T025 [US1] Implement session expiry handling in AuthProvider: on 401 response, redirect to /auth/signin with returnUrl query param; after successful sign-in, redirect back to preserved URL

**Checkpoint**: User Story 1 fully functional — students can register, sign in, sign out, reset password, view/edit profile, and delete account. Navbar reflects auth state.

---

## Phase 4: User Story 2 — Complete Background Assessment (Priority: P2)

**Goal**: Students complete a 5-question background assessment after signup. Responses are saved, a computed level (beginner/intermediate/advanced) is derived, and a starting chapter recommendation is generated. Assessment is editable from the profile.

**Independent Test**: Sign in, complete assessment with all fields, verify responses saved. Change assessment answers, verify computed level updates. Skip assessment during signup, verify intermediate defaults applied.

**Dependencies**: Requires US1 (authenticated user exists)

### Implementation for User Story 2

- [X] T026 [US2] Implement assessment service: computeLevel algorithm (count advanced indicators → beginner/intermediate/advanced), generateStartingRecommendation (map level to chapter slug), upsert assessment logic (create or update), validate enum inputs in backend/src/services/assessment.service.ts
- [X] T027 [US2] Implement assessment routes: GET /api/assessment returns current assessment or 404; POST /api/assessment creates/updates assessment, computes level, generates recommendation, returns both per contracts/auth-api.yaml in backend/src/routes/assessment.ts
- [X] T028 [P] [US2] Create AssessmentForm component with 5 questions: dev experience (radio: beginner/intermediate/advanced), Python proficiency (radio: none/basic/proficient/expert), robotics background (radio: none/hobbyist/professional), ROS exposure (radio: none/ros1/ros2), learning goals (multi-select checkboxes: simulation, perception, navigation, voice_control, full_stack_robotics) in frontend/src/components/AssessmentForm.tsx
- [X] T029 [US2] Create standalone assessment page wrapping AssessmentForm in ProtectedRoute; on submit call POST /api/assessment; display starting recommendation result; include "Skip for now" link in frontend/src/pages/assessment.tsx
- [X] T030 [US2] Update signup page (T017) to redirect to /assessment after successful registration instead of /dashboard; assessment page shows "Skip" option linking to /dashboard
- [X] T031 [US2] Add assessment section to profile page (T023): display current assessment responses and computed level; include "Retake Assessment" button that shows AssessmentForm inline

**Checkpoint**: User Story 2 fully functional — students complete assessment, see computed level and starting recommendation. Assessment data persists and is editable from profile.

---

## Phase 5: User Story 3 — Track Chapter Progress (Priority: P3)

**Goal**: Students see their progress through the textbook. Chapters can be marked as complete. A dashboard shows per-module and overall progress. "Continue where you left off" links to the last active chapter. Sidebar shows completion indicators.

**Independent Test**: Sign in, navigate to a chapter, mark it as in_progress then completed. Visit dashboard, verify completion shows. Sign out and back in, verify "Continue where you left off" shows the correct chapter. Check sidebar for checkmark on completed chapter.

**Dependencies**: Requires US1 (authenticated user). Independent of US2.

### Implementation for User Story 3

- [X] T032 [US3] Implement progress service: getProgressForUser (all chapters with summary), getChapterProgress (single chapter), updateChapterProgress (status transition validation: not_started→in_progress→completed), getContinueChapter (most recent in_progress by lastAccessedAt), computeProgressSummary (per-module completion percentage, overall percentage) in backend/src/services/progress.service.ts
- [X] T033 [US3] Implement progress routes: GET /api/progress returns all progress with summary; GET /api/progress/continue returns last active chapter; PATCH /api/progress/:chapterId updates status per contracts/auth-api.yaml in backend/src/routes/progress.ts
- [X] T034 [P] [US3] Create ProgressBar component: accepts moduleProgress array; renders horizontal bars per module with percentage labels; renders overall course progress bar in frontend/src/components/ProgressBar.tsx
- [X] T035 [US3] Create dashboard page wrapped in ProtectedRoute: fetch GET /api/progress on mount; display "Continue where you left off" card (from GET /api/progress/continue) at top; render ProgressBar for each module; show list of chapters with status indicators in frontend/src/pages/dashboard.tsx
- [X] T036 [US3] Swizzle DocItem Content component to add "Mark as Complete" button at the bottom of chapter pages for authenticated users; on click call PATCH /api/progress/:chapterId with status=completed; also call PATCH with status=in_progress on page load (tracks access) in frontend/src/theme/DocItem/Content/index.tsx
- [X] T037 [US3] Update dashboard (T035) to show "Continue where you left off" card prominently: display chapter title, last accessed date, and direct link; show on initial sign-in redirect
- [X] T038 [US3] Add visual completion indicators to Docusaurus sidebar: swizzle or extend sidebar item component to show a checkmark icon next to completed chapters and a progress dot next to in-progress chapters based on cached progress data from AuthProvider

**Checkpoint**: User Story 3 fully functional — progress tracked per chapter, dashboard shows per-module completion, sidebar shows checkmarks, "Continue where you left off" works across sessions.

---

## Phase 6: User Story 4 — Experience Personalized Content Depth (Priority: P4)

**Goal**: Chapter content adapts default visibility based on the student's computed assessment level. Beginner sections expand for beginners, collapse for advanced users. Manual toggle always works. Intermediate is the default for unassessed users.

**Independent Test**: Complete assessment as beginner, open chapter with `<PersonalizedDetails>` sections, verify beginner sections expanded. Change assessment to advanced, reload, verify sections collapsed. Manually toggle a section, verify it stays toggled regardless of default.

**Dependencies**: Requires US1 (auth) and US2 (assessment data for computed level)

### Implementation for User Story 4

- [X] T039 [US4] Create PersonalizedDetails component: reads computedLevel from useAuth hook; renders HTML details element with data-level attribute; sets initial open state based on level match (beginner level → beginner sections open, advanced level → beginner sections closed); defaults to intermediate behavior when no assessment; allows manual toggle override in frontend/src/components/PersonalizedDetails.tsx
- [X] T040 [US4] Integrate content personalization into DocItem Content swizzle (T036): on chapter page load, read user's computedLevel from auth context; apply default open/closed state to all PersonalizedDetails elements on the page within 1 second of load (SC-005) in frontend/src/theme/DocItem/Content/index.tsx
- [X] T041 [US4] Verify manual expand/collapse override: ensure clicking a PersonalizedDetails element toggles its state independently of the computed default; user's manual choice persists during the page session (FR-015)

**Checkpoint**: User Story 4 fully functional — content depth adapts based on assessment level, manual override works, intermediate default applied for unassessed users.

---

## Phase 7: User Story 5 — View Recommended Learning Path (Priority: P5)

**Goal**: Students see a personalized learning path based on their assessment learning goals. The path highlights priority modules, suggests chapter ordering, and reflects completed chapters. Path regenerates when assessment updates.

**Independent Test**: Complete assessment with "simulation" learning goal, view learning path, verify Modules 2 and 3 highlighted with Module 1 as prerequisite. Complete a chapter, verify path shows it as done. Change goals to "voice control", verify path updates to highlight Module 4.

**Dependencies**: Requires US1 (auth), US2 (assessment with learning goals), and US3 (progress data for chapter status overlay)

### Implementation for User Story 5

- [X] T042 [US5] Implement learning path service: generatePath from assessment (apply rule table: simulation→[2,3], perception→[3,1], navigation→[3,1], voice_control→[4,1], full_stack_robotics→[1,2,3,4]; Module 1 always prerequisite); compute recommended chapter order by module priority; merge with user's progress to set chapter status; determine starting chapter in backend/src/services/learning-path.service.ts
- [X] T043 [US5] Implement learning path routes: GET /api/learning-path returns current path merged with progress (404 if no assessment); POST /api/learning-path regenerates path from current assessment per contracts/auth-api.yaml in backend/src/routes/learning-path.ts
- [X] T044 [P] [US5] Create LearningPathView component: display prioritized chapters grouped by module; each chapter shows title, module, priority (required/recommended/optional), and status (not_started/in_progress/completed); highlight priority modules; link each chapter to its page in frontend/src/components/LearningPathView.tsx
- [X] T045 [US5] Integrate LearningPathView into dashboard page (T035): add "Your Learning Path" section below progress overview; fetch GET /api/learning-path and pass data to component; show "Complete your assessment to get a personalized path" message if 404
- [X] T046 [US5] Trigger learning path regeneration when assessment is updated: in assessment route POST handler (T027), after saving assessment, call learning path service generatePath and upsert the result; ensures path stays in sync with assessment changes

**Checkpoint**: User Story 5 fully functional — personalized learning path generated from assessment goals, reflects progress, updates when goals change.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Security hardening, accessibility, and production readiness across all stories

- [X] T047 [P] Add rate limiting middleware to Hono app: limit auth endpoints to 10 requests/minute per IP; limit custom API endpoints to 60 requests/minute per user in backend/src/index.ts
- [X] T048 [P] Configure production CORS: restrict allowed origins to production domain; ensure credentials: true for cookie-based auth in backend/src/index.ts
- [X] T049 Implement soft-delete account deletion: Better-Auth delete-user hook sets deletedAt timestamp instead of hard delete; add scheduled purge logic that deletes all data for users where deletedAt < NOW() - 30 days (learning_path → chapter_progress → background_assessment → session → account → user) in backend/src/auth/index.ts
- [ ] T050 Validate all auth and assessment forms meet WCAG accessibility: proper label associations, focus management, keyboard navigation, error announcements via aria-live, color-independent status indicators across all pages in frontend/src/pages/auth/ and frontend/src/pages/assessment.tsx
- [ ] T051 Run quickstart.md validation: follow setup steps from quickstart.md end to end, verify backend starts, database schema pushes, frontend connects, signup/signin works, assessment saves

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion — BLOCKS all user stories
- **US1 (Phase 3)**: Depends on Foundational — No dependencies on other stories
- **US2 (Phase 4)**: Depends on Foundational + requires US1 for authenticated user
- **US3 (Phase 5)**: Depends on Foundational + requires US1 for authenticated user. Independent of US2
- **US4 (Phase 6)**: Depends on US1 + US2 (needs assessment computed level)
- **US5 (Phase 7)**: Depends on US1 + US2 + US3 (needs assessment goals + progress data)
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

```
Phase 1: Setup
    │
    ▼
Phase 2: Foundational (BLOCKS ALL)
    │
    ├──▶ Phase 3: US1 - Sign Up/Sign In (P1) ──── MVP ────┐
    │                                                       │
    │    ┌──────────────────────────────────────────────────┘
    │    │
    │    ├──▶ Phase 4: US2 - Assessment (P2) ──────────────┐
    │    │                                                  │
    │    ├──▶ Phase 5: US3 - Progress (P3) ────────────────┤
    │    │                                                  │
    │    │    ┌─────────────────────────────────────────────┘
    │    │    │
    │    │    ├──▶ Phase 6: US4 - Content Depth (P4) [needs US2]
    │    │    │
    │    │    └──▶ Phase 7: US5 - Learning Path (P5) [needs US2 + US3]
    │    │
    │    └──▶ Phase 8: Polish
```

### Within Each User Story

- Models/Services before Routes
- Backend routes before Frontend pages
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- **Phase 1**: T002, T003, T004 can run in parallel
- **Phase 2**: T006, T007 in parallel; T009, T013, T014, T016 in parallel (different files)
- **Phase 3**: T017, T018, T019, T020 in parallel (different page files)
- **Phase 4**: T028 in parallel with T026 (component vs service, different projects)
- **Phase 5**: T034 in parallel with T032 (component vs service, different projects)
- **Phase 7**: T044 in parallel with T042 (component vs service, different projects)
- **Phase 8**: T047, T048 in parallel (different concerns in same file but independent changes)
- **Cross-phase**: After US1 completes, US2 and US3 can proceed in parallel

---

## Parallel Example: Phase 3 (US1)

```bash
# Launch all auth page components in parallel (different files, no dependencies):
Task T017: "Create sign up page in frontend/src/pages/auth/signup.tsx"
Task T018: "Create sign in page in frontend/src/pages/auth/signin.tsx"
Task T019: "Create forgot password page in frontend/src/pages/auth/forgot-password.tsx"
Task T020: "Create reset password page in frontend/src/pages/auth/reset-password.tsx"

# Then sequentially:
Task T021: "Swizzle Navbar" (depends on auth pages existing for links)
Task T022: "Profile API routes" (backend, could parallel with T021)
Task T023: "Profile page" (depends on T022 for API)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T016) — CRITICAL
3. Complete Phase 3: US1 Sign Up/Sign In (T017-T025)
4. **STOP and VALIDATE**: Test signup → signin → signout → password reset → profile edit → account delete
5. Deploy backend + frontend — students can create accounts

### Incremental Delivery

1. Setup + Foundational → Infrastructure ready
2. Add US1 → Students create accounts → **Deploy (MVP!)**
3. Add US2 → Students complete assessment, see computed level → Deploy
4. Add US3 → Students track progress, see dashboard → Deploy
5. Add US4 → Content depth adapts to student level → Deploy
6. Add US5 → Personalized learning paths → Deploy (Feature Complete!)
7. Polish → Rate limiting, accessibility, production CORS → Deploy (Production Ready)

### Suggested First Implementation Session

Focus on Phases 1-3 (T001-T025) to deliver a working MVP:
- ~4 setup tasks
- ~12 foundational tasks
- ~9 user story 1 tasks
- **Total: 25 tasks for MVP**

---

## Requirements Traceability

| Requirement | Task(s) | User Story |
|-------------|---------|------------|
| FR-001 | T017 | US1 |
| FR-002 | T024 | US1 |
| FR-003 | T018 | US1 |
| FR-004 | T019, T020 | US1 |
| FR-005 | T008, T011 | Foundational |
| FR-006 | T021 | US1 |
| FR-007 | T028 | US2 |
| FR-008 | T026, T027 | US2 |
| FR-009 | T026 | US2 |
| FR-010 | T032, T033 | US3 |
| FR-011 | T035 | US3 |
| FR-012 | T037 | US3 |
| FR-013 | T038 | US3 |
| FR-014 | T039, T040 | US4 |
| FR-015 | T041 | US4 |
| FR-016 | T042, T043 | US5 |
| FR-017 | T022, T023, T031 | US1, US2 |
| FR-018 | T023, T049 | US1, Polish |
| FR-019 | T032 (server-side) | US3 |
| FR-020 | T008 (Better-Auth) | Foundational |

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable after its dependencies
- Commit after each task or logical group
- Stop at any checkpoint to validate the story independently
- The Docusaurus frontend project (Feature 001) must be initialized before frontend tasks — if it doesn't exist yet, T004 should include Docusaurus initialization
