# Data Model: User Authentication & Content Personalization

**Feature Branch**: `003-user-auth-personalization`
**Date**: 2026-02-10
**ORM**: Drizzle ORM | **Database**: Neon Serverless Postgres

---

## Entity Relationship Diagram

```
┌──────────────┐       ┌──────────────────┐       ┌──────────────────┐
│    user       │──1:1──│   account         │       │    session        │
│              │       │                  │       │                  │
│ id (PK)      │       │ id (PK)          │       │ id (PK)          │
│ name         │──1:N──│ userId (FK)       │       │ userId (FK)      │
│ email (UQ)   │       │ accountId         │       │ token (UQ)       │
│ emailVerified│       │ providerId        │       │ expiresAt        │
│ image        │       │ password (hashed) │       │ ipAddress        │
│ displayName  │       │ createdAt         │       │ userAgent        │
│ createdAt    │       │ updatedAt         │       │ createdAt        │
│ updatedAt    │       └──────────────────┘       │ updatedAt        │
│ deletedAt    │                                   └──────────────────┘
└──────┬───────┘
       │
       ├──1:1──┌──────────────────────────┐
       │       │  background_assessment    │
       │       │                          │
       │       │ id (PK)                  │
       │       │ userId (FK, UQ)          │
       │       │ devExperience            │
       │       │ pythonProficiency        │
       │       │ roboticsBackground       │
       │       │ rosExposure              │
       │       │ learningGoals (JSON)     │
       │       │ computedLevel            │
       │       │ completedAt              │
       │       │ updatedAt               │
       │       └──────────────────────────┘
       │
       ├──1:N──┌──────────────────────────┐
       │       │  chapter_progress         │
       │       │                          │
       │       │ id (PK)                  │
       │       │ userId (FK)              │
       │       │ chapterId               │
       │       │ status                   │
       │       │ startedAt               │
       │       │ completedAt             │
       │       │ lastAccessedAt          │
       │       │ UQ(userId, chapterId)   │
       │       └──────────────────────────┘
       │
       └──1:1──┌──────────────────────────┐
               │  learning_path            │
               │                          │
               │ id (PK)                  │
               │ userId (FK, UQ)          │
               │ recommendedChapters (JSON)│
               │ priorityModules (JSON)   │
               │ startingChapter          │
               │ generatedAt             │
               │ assessmentVersion        │
               └──────────────────────────┘
```

---

## Table Definitions

### `user` (managed by Better-Auth + custom fields)

Better-Auth auto-creates the `user` table. We extend it with `displayName` and `deletedAt`.

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | Better-Auth generated ID |
| `name` | `text` | NOT NULL | Full name (Better-Auth core field) |
| `email` | `text` | NOT NULL, UNIQUE | User email address |
| `emailVerified` | `boolean` | DEFAULT false | Email verification status |
| `image` | `text` | NULLABLE | Profile image URL |
| `displayName` | `text` | NOT NULL | Display name shown in UI (custom field) |
| `createdAt` | `timestamp` | NOT NULL, DEFAULT now() | Account creation timestamp |
| `updatedAt` | `timestamp` | NOT NULL | Last profile update |
| `deletedAt` | `timestamp` | NULLABLE | Soft-delete timestamp (NULL = active) |

### `session` (managed by Better-Auth)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | Session identifier |
| `userId` | `text` | FK → user.id, NOT NULL | Owning user |
| `token` | `text` | NOT NULL, UNIQUE | Session token (stored in HTTP-only cookie) |
| `expiresAt` | `timestamp` | NOT NULL | Session expiry time |
| `ipAddress` | `text` | NULLABLE | Client IP address |
| `userAgent` | `text` | NULLABLE | Client user agent string |
| `createdAt` | `timestamp` | NOT NULL, DEFAULT now() | Session creation time |
| `updatedAt` | `timestamp` | NOT NULL | Last activity update |

### `account` (managed by Better-Auth)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | Account identifier |
| `userId` | `text` | FK → user.id, NOT NULL | Owning user |
| `accountId` | `text` | NOT NULL | Provider account ID |
| `providerId` | `text` | NOT NULL | Auth provider ("credential" for email/pass) |
| `password` | `text` | NULLABLE | Bcrypt-hashed password |
| `createdAt` | `timestamp` | NOT NULL | Account creation time |
| `updatedAt` | `timestamp` | NOT NULL | Last update time |

### `verification` (managed by Better-Auth)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | Verification record ID |
| `identifier` | `text` | NOT NULL | Email or token identifier |
| `value` | `text` | NOT NULL | Verification token value |
| `expiresAt` | `timestamp` | NOT NULL | Token expiry |
| `createdAt` | `timestamp` | NOT NULL | Creation time |
| `updatedAt` | `timestamp` | NOT NULL | Last update time |

### `background_assessment` (custom)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | UUID |
| `userId` | `text` | FK → user.id, UNIQUE, NOT NULL | One assessment per user |
| `devExperience` | `text` | NOT NULL | Enum: `beginner`, `intermediate`, `advanced` |
| `pythonProficiency` | `text` | NOT NULL | Enum: `none`, `basic`, `proficient`, `expert` |
| `roboticsBackground` | `text` | NOT NULL | Enum: `none`, `hobbyist`, `professional` |
| `rosExposure` | `text` | NOT NULL | Enum: `none`, `ros1`, `ros2` |
| `learningGoals` | `jsonb` | NOT NULL | Array of strings from: `simulation`, `perception`, `navigation`, `voice_control`, `full_stack_robotics` |
| `computedLevel` | `text` | NOT NULL | Derived level: `beginner`, `intermediate`, `advanced` |
| `completedAt` | `timestamp` | NOT NULL | First completion time |
| `updatedAt` | `timestamp` | NOT NULL | Last update time |

**Computed Level Algorithm**:
- Count "advanced" indicators: `devExperience=advanced`, `pythonProficiency=expert`, `roboticsBackground=professional`, `rosExposure=ros2`
- 0-1 advanced indicators → `beginner`
- 2 advanced indicators → `intermediate`
- 3-4 advanced indicators → `advanced`

### `chapter_progress` (custom)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | UUID |
| `userId` | `text` | FK → user.id, NOT NULL | Owning user |
| `chapterId` | `text` | NOT NULL | Chapter slug (e.g., `intro-to-ros2`) |
| `status` | `text` | NOT NULL, DEFAULT `not_started` | Enum: `not_started`, `in_progress`, `completed` |
| `startedAt` | `timestamp` | NULLABLE | First access time |
| `completedAt` | `timestamp` | NULLABLE | Completion time |
| `lastAccessedAt` | `timestamp` | NOT NULL | Last access time |

**Unique constraint**: `(userId, chapterId)` — one progress record per user per chapter.

**State Transitions**:
```
not_started ──▶ in_progress ──▶ completed
                     │                │
                     └────────────────┘ (re-read allowed)
```

### `learning_path` (custom)

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | `text` | PK | UUID |
| `userId` | `text` | FK → user.id, UNIQUE, NOT NULL | One path per user |
| `recommendedChapters` | `jsonb` | NOT NULL | Ordered array of chapter slugs |
| `priorityModules` | `jsonb` | NOT NULL | Array of priority module IDs (1-4) |
| `startingChapter` | `text` | NOT NULL | Recommended first chapter slug |
| `generatedAt` | `timestamp` | NOT NULL | Path generation time |
| `assessmentVersion` | `integer` | NOT NULL, DEFAULT 1 | Assessment data version used |

---

## Indexes

| Table | Index | Columns | Purpose |
|-------|-------|---------|---------|
| `user` | `idx_user_email` | `email` | Login lookup (Better-Auth managed) |
| `user` | `idx_user_deleted` | `deletedAt` | Filter active users |
| `session` | `idx_session_token` | `token` | Session validation (Better-Auth managed) |
| `session` | `idx_session_userId` | `userId` | User's active sessions |
| `background_assessment` | `idx_assessment_userId` | `userId` | Assessment lookup by user |
| `chapter_progress` | `idx_progress_userId` | `userId` | All progress for a user |
| `chapter_progress` | `idx_progress_userId_chapter` | `userId, chapterId` | Specific chapter progress (unique) |
| `chapter_progress` | `idx_progress_lastAccessed` | `userId, lastAccessedAt DESC` | "Continue where you left off" query |
| `learning_path` | `idx_path_userId` | `userId` | Path lookup by user |

---

## Cascade Rules

| Parent | Child | On Delete |
|--------|-------|-----------|
| `user` | `session` | CASCADE (Better-Auth managed) |
| `user` | `account` | CASCADE (Better-Auth managed) |
| `user` | `background_assessment` | CASCADE |
| `user` | `chapter_progress` | CASCADE |
| `user` | `learning_path` | CASCADE |

---

## Data Retention

- **Active users**: All data retained indefinitely
- **Soft-deleted users** (`deletedAt` IS NOT NULL): Data marked for purge
- **Purge schedule**: Background job runs daily, deletes all data for users where `deletedAt < NOW() - 30 days`
- **Purge order**: learning_path → chapter_progress → background_assessment → session → account → user (respects foreign keys)
