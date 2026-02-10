# Quickstart: User Authentication & Content Personalization

**Feature Branch**: `003-user-auth-personalization`
**Date**: 2026-02-10

---

## Prerequisites

- Node.js 20+ installed
- A Neon Serverless Postgres database (free tier: [neon.tech](https://neon.tech))
- A Resend account for email delivery (free tier: [resend.com](https://resend.com))

---

## 1. Project Setup

The auth backend lives alongside the Docusaurus site in the repository root:

```bash
# From repository root (physical_ai_book/)
cd backend
npm install
```

### Dependencies

```json
{
  "dependencies": {
    "better-auth": "^1.3.0",
    "hono": "^4.0.0",
    "@hono/node-server": "^1.0.0",
    "drizzle-orm": "^0.40.0",
    "@neondatabase/serverless": "^0.10.0",
    "resend": "^4.0.0"
  },
  "devDependencies": {
    "drizzle-kit": "^0.30.0",
    "typescript": "^5.5.0",
    "vitest": "^3.0.0",
    "@types/node": "^22.0.0"
  }
}
```

---

## 2. Environment Variables

Create `.env` at the backend root:

```env
# Database (Neon)
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# Better-Auth
BETTER_AUTH_SECRET=your-random-secret-at-least-32-chars
BETTER_AUTH_URL=http://localhost:3001

# Resend (Email)
RESEND_API_KEY=re_xxxxxxxxxx

# CORS (Docusaurus dev server)
FRONTEND_URL=http://localhost:3000
```

---

## 3. Database Schema

Push the schema to Neon:

```bash
npx drizzle-kit push
```

This creates the tables: `user`, `session`, `account`, `verification`, `background_assessment`, `chapter_progress`, `learning_path`.

---

## 4. Run the Auth Backend

```bash
npm run dev
# Auth API running at http://localhost:3001
```

---

## 5. Docusaurus Client Setup

In the Docusaurus frontend (`frontend/`):

```bash
npm install better-auth
```

Create the auth client:

```typescript
// frontend/src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3001", // Backend URL
});
```

Wrap the site with auth context:

```tsx
// frontend/src/theme/Root.tsx
import React from "react";
import { AuthProvider } from "../components/AuthProvider";

export default function Root({ children }: { children: React.ReactNode }) {
  return <AuthProvider>{children}</AuthProvider>;
}
```

---

## 6. Verify It Works

1. Start the backend: `cd backend && npm run dev`
2. Start Docusaurus: `cd frontend && npm start`
3. Navigate to `http://localhost:3000/auth/signup`
4. Create an account
5. Check the Neon dashboard — you should see a row in the `user` table

---

## Key API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/auth/sign-up/email` | POST | Register new user |
| `/api/auth/sign-in/email` | POST | Sign in |
| `/api/auth/sign-out` | POST | Sign out |
| `/api/auth/get-session` | GET | Get current session |
| `/api/auth/forget-password` | POST | Request password reset |
| `/api/auth/reset-password` | POST | Reset password with token |
| `/api/assessment` | GET/POST | Read/write background assessment |
| `/api/progress` | GET | Get all chapter progress |
| `/api/progress/{chapterId}` | PATCH | Update chapter status |
| `/api/progress/continue` | GET | Get "continue where you left off" |
| `/api/learning-path` | GET/POST | Read/regenerate learning path |
| `/api/profile` | GET/PATCH/DELETE | Read/update/soft-delete user profile |
| `/api/health` | GET | Health check |

---

## Development Workflow

1. Schema changes → Edit Drizzle schema → `npx drizzle-kit push`
2. API changes → Edit Hono routes → Auto-restart with dev server
3. Frontend changes → Edit Docusaurus components → Hot reload
4. Run tests: `cd backend && npm test` (Vitest)
