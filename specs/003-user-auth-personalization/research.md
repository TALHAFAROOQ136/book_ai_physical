# Research: User Authentication & Content Personalization

**Feature Branch**: `003-user-auth-personalization`
**Date**: 2026-02-10
**Status**: Complete

---

## Research Task 1: Authentication Framework — Better-Auth

### Decision: Better-Auth v1.3+ with email/password authentication

### Rationale
- Constitution mandates Better-Auth integration (§ Authentication & User Management)
- Framework-agnostic TypeScript library with first-class React client (`better-auth/react`)
- Built-in email/password auth, password reset flow, session management, and account deletion
- Drizzle ORM adapter available for PostgreSQL (Neon Serverless)
- Supports custom user fields via `additionalFields` — ideal for storing assessment metadata
- Plugin system for future extensibility (admin, anonymous, 2FA)
- Session-based auth with HTTP-only cookies out of the box (matching spec assumption)

### Alternatives Considered
| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **Better-Auth** | Constitution-mandated, TypeScript-native, Drizzle adapter, React client, built-in password reset | Newer library, smaller ecosystem than Auth.js | **Selected** |
| **Auth.js (NextAuth)** | Mature ecosystem, many providers | Designed for Next.js, awkward with Docusaurus, not constitution-mandated | Rejected |
| **Lucia Auth** | Lightweight, good DX | Deprecated (author recommends rolling your own) | Rejected |
| **Custom JWT** | Full control | Significant security surface to maintain, reinventing solved problems | Rejected |

### Key Integration Pattern
```typescript
// Server: Better-Auth instance with Drizzle adapter
import { betterAuth } from "better-auth";
import { drizzleAdapter } from "better-auth/adapters/drizzle";

export const auth = betterAuth({
  database: drizzleAdapter(db, { provider: "pg" }),
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    sendResetPassword: async ({ user, url }) => { /* Resend email */ },
  },
  user: {
    additionalFields: { displayName: { type: "string", required: true } },
  },
});

// Client: React hooks for Docusaurus
import { createAuthClient } from "better-auth/react";
export const authClient = createAuthClient();
```

---

## Research Task 2: Database — Neon Serverless Postgres + Drizzle ORM

### Decision: Neon Serverless Postgres with Drizzle ORM

### Rationale
- Constitution mandates Neon Serverless Postgres (§ Technical Implementation)
- Free tier supports up to 0.5 GB storage and 190 compute hours/month — sufficient for textbook scale
- Serverless driver (`@neondatabase/serverless`) provides HTTP-based connections — no persistent connection needed
- Drizzle ORM is the recommended adapter for Better-Auth with PostgreSQL
- Drizzle provides type-safe schema definitions, migrations, and query builder
- Schema-first approach aligns with SDD methodology

### Alternatives Considered
| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **Neon + Drizzle** | Constitution-mandated, serverless, free tier, type-safe ORM | Requires Neon account setup | **Selected** |
| **Supabase Postgres** | Built-in auth (but we use Better-Auth), generous free tier | Redundant auth layer, not constitution-mandated | Rejected |
| **PlanetScale (MySQL)** | Serverless, branching | MySQL not PostgreSQL, pricing changes | Rejected |
| **SQLite (Turso)** | Simple, edge-native | Less suitable for multi-device sync, no built-in Better-Auth adapter parity | Rejected |

---

## Research Task 3: Backend Runtime — Node.js with Hono

### Decision: Hono framework on Node.js for Better-Auth API handler

### Rationale
- Better-Auth requires a Node.js/TypeScript runtime (it's a TypeScript library)
- Hono is a lightweight, fast web framework that works on Node.js, Cloudflare Workers, and Vercel Edge
- Better-Auth has a standard handler that can mount on any framework (Express, Hono, Fastify)
- Hono provides clean middleware pattern for additional API routes (assessment, progress, learning path)
- Keeps the auth backend lightweight and deployable as a serverless function
- Separates cleanly from the Python/FastAPI RAG chatbot backend (Feature 002)

### Alternatives Considered
| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **Hono** | Ultra-lightweight, universal runtime, clean middleware, TypeScript-first | Newer, smaller ecosystem | **Selected** |
| **Express** | Most popular Node.js framework, huge ecosystem | Heavier, callback-based API, TypeScript support is addon | Viable alternative |
| **Fastify** | Fast, schema validation, TypeScript support | Heavier than needed for auth API wrapper | Rejected |
| **Next.js API Routes** | Full-stack solution | Overkill — Docusaurus is our frontend, not Next.js | Rejected |

### Architecture Pattern
```
┌─────────────────────┐     ┌──────────────────────────────┐
│  Docusaurus (React)  │────▶│  Auth API (Hono + Better-Auth)│
│  Static Site (SSG)   │     │  /api/auth/* (Better-Auth)    │
│  Custom Pages        │     │  /api/assessment/*            │
│  Theme Components    │     │  /api/progress/*              │
│                      │     │  /api/learning-path/*         │
└─────────────────────┘     │  /api/profile/*               │
                            └──────────┬───────────────────┘
                                       │
                            ┌──────────▼───────────────────┐
                            │  Neon Serverless Postgres     │
                            │  (Drizzle ORM)                │
                            └──────────────────────────────┘
```

---

## Research Task 4: Email Service — Resend

### Decision: Resend for transactional email delivery

### Rationale
- Clean API with TypeScript SDK — pairs well with Better-Auth's `sendResetPassword` callback
- Free tier: 100 emails/day, 3,000 emails/month — sufficient for password reset volume
- Simple integration: single function call per email
- Better-Auth documentation commonly demonstrates Resend integration
- Domain verification for production; sandbox mode for development

### Alternatives Considered
| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **Resend** | Clean TS SDK, generous free tier, Better-Auth integration examples | Newer service | **Selected** |
| **SendGrid** | Mature, large free tier | Complex API, heavy SDK | Rejected |
| **AWS SES** | Cheapest at scale | Requires AWS account, complex setup | Rejected |
| **Mailgun** | Reliable | No free tier anymore | Rejected |

---

## Research Task 5: Docusaurus Integration Pattern

### Decision: Theme swizzling + custom pages + React Context

### Rationale
- Docusaurus supports a `src/theme/Root.js` component for wrapping the entire site with React Context providers — ideal for auth state
- Custom React pages at `src/pages/` for signup, signin, dashboard, profile, assessment
- Theme component swizzling for Navbar (add Sign In/Out buttons) and DocItem (content personalization)
- `better-auth/react` client provides `useSession()` hook for reactive auth state
- Content depth personalization via client-side JavaScript toggling `<details>` element `open` attribute based on user's assessment level

### Integration Points
1. **`src/theme/Root.js`**: AuthProvider wrapping entire site with session context
2. **`src/theme/Navbar/`**: Swizzled navbar with auth buttons (Sign In / User Menu)
3. **`src/theme/DocItem/`**: Swizzled doc item wrapper for content personalization
4. **`src/pages/auth/`**: Custom pages for signin, signup, forgot-password, reset-password
5. **`src/pages/dashboard.tsx`**: Progress dashboard with module completion
6. **`src/pages/profile.tsx`**: User profile editing and assessment
7. **`src/pages/assessment.tsx`**: Background assessment form
8. **`src/components/`**: Reusable components (ProgressBar, AssessmentForm, LearningPath)

---

## Research Task 6: Content Personalization Approach

### Decision: Client-side `<details>` element toggling based on assessment level

### Rationale
- Spec assumes content depth via `<details>` elements in MDX (§ Assumptions)
- No server-side rendering needed — Docusaurus is a static site generator
- On page load, client JavaScript reads user's assessment level from auth context
- Sets `open` attribute on tagged `<details>` elements based on level:
  - Beginner: all `<details data-level="beginner">` expanded by default
  - Advanced: beginner sections collapsed, advanced sections expanded
  - Intermediate (default): standard state
- Users can always manually toggle regardless of defaults (FR-015)
- Zero impact on build time or static generation — purely client-side enhancement

### Implementation Pattern
- MDX chapters use custom `<PersonalizedDetails>` React component
- Component reads user level from AuthContext
- Sets initial `open` state based on level
- Falls back to intermediate if no assessment data

---

## Research Task 7: Learning Path Recommendation Engine

### Decision: Rule-based mapping from learning goals to module priorities

### Rationale
- Spec explicitly states rule-based mapping, not ML (§ Assumptions)
- 5 learning goals map cleanly to 4 modules:
  - Simulation → Module 2 (Gazebo/Unity), Module 3 (Isaac)
  - Perception → Module 3 (Isaac), Module 1 (ROS 2 sensors)
  - Navigation → Module 3 (Isaac Nav2), Module 1 (ROS 2)
  - Voice Control → Module 4 (VLA), Module 1 (ROS 2)
  - Full-Stack Robotics → All modules, sequential
- Module 1 (ROS 2) is always a prerequisite
- Path generation: prioritize selected goals, order by dependency, mark completed chapters

### Rule Table
| Learning Goal | Priority Modules | Prerequisite |
|---------------|-----------------|--------------|
| Simulation | 2, 3 | 1 |
| Perception | 3, 1 | 1 |
| Navigation | 3, 1 | 1 |
| Voice Control | 4, 1 | 1 |
| Full-Stack Robotics | 1, 2, 3, 4 | None (sequential) |

---

## Research Task 8: Session & Security Architecture

### Decision: Better-Auth built-in session management with HTTP-only cookies

### Rationale
- Better-Auth provides session management out of the box
- Sessions stored in database (Neon Postgres) via `session` table
- HTTP-only, Secure, SameSite cookies prevent XSS and CSRF
- Session expiry configurable (default: 7 days, extended with "Remember Me")
- CORS configuration needed since Docusaurus (static) and API (Hono) run on different origins in development
- Production: same domain with API on subdomain (e.g., `api.textbook.example.com`) or path prefix

### Security Measures
- Passwords hashed with bcrypt (Better-Auth default)
- Password strength: minimum 8 characters, at least 1 letter + 1 number (FR-002)
- Rate limiting on auth endpoints (Better-Auth plugin or Hono middleware)
- CSRF protection via SameSite cookie policy
- No secrets in client code — all auth logic server-side
- Environment variables for all credentials (`.env` pattern per Constitution Principle 8)

---

## Summary: All NEEDS CLARIFICATION Resolved

| Unknown | Resolution |
|---------|-----------|
| Language/Version | TypeScript (Node.js 20+) for auth backend |
| Primary Dependencies | Better-Auth, Hono, Drizzle ORM |
| Storage | Neon Serverless Postgres |
| Testing | Vitest (unit/integration), Playwright (e2e) |
| Target Platform | Serverless deployment (Vercel/Railway/Render) |
| Project Type | Web application (Docusaurus frontend + Hono API backend) |
| Performance Goals | Auth endpoints <200ms p95, page load impact <200ms |
| Constraints | Free tier databases, <500ms total auth overhead |
| Scale/Scope | ~1000 concurrent users, ~10,000 registered users |
| Email Service | Resend (free tier: 100/day) |
| ORM | Drizzle ORM with PostgreSQL adapter |
| Session Store | Database-backed via Better-Auth |
