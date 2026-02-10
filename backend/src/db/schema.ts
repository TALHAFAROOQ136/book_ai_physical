import {
  pgTable,
  text,
  timestamp,
  boolean,
  integer,
  jsonb,
  uniqueIndex,
  index,
} from "drizzle-orm/pg-core";

// ─── Better-Auth Managed Tables ──────────────────────────────────────

export const user = pgTable(
  "user",
  {
    id: text("id").primaryKey(),
    name: text("name").notNull(),
    email: text("email").notNull().unique(),
    emailVerified: boolean("email_verified").default(false).notNull(),
    image: text("image"),
    displayName: text("display_name").notNull(),
    createdAt: timestamp("created_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at").defaultNow().notNull(),
    deletedAt: timestamp("deleted_at"),
  },
  (table) => [
    index("idx_user_deleted").on(table.deletedAt),
  ]
);

export const session = pgTable("session", {
  id: text("id").primaryKey(),
  userId: text("user_id")
    .notNull()
    .references(() => user.id, { onDelete: "cascade" }),
  token: text("token").notNull().unique(),
  expiresAt: timestamp("expires_at").notNull(),
  ipAddress: text("ip_address"),
  userAgent: text("user_agent"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const account = pgTable("account", {
  id: text("id").primaryKey(),
  userId: text("user_id")
    .notNull()
    .references(() => user.id, { onDelete: "cascade" }),
  accountId: text("account_id").notNull(),
  providerId: text("provider_id").notNull(),
  accessToken: text("access_token"),
  refreshToken: text("refresh_token"),
  accessTokenExpiresAt: timestamp("access_token_expires_at"),
  refreshTokenExpiresAt: timestamp("refresh_token_expires_at"),
  scope: text("scope"),
  idToken: text("id_token"),
  password: text("password"),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

export const verification = pgTable("verification", {
  id: text("id").primaryKey(),
  identifier: text("identifier").notNull(),
  value: text("value").notNull(),
  expiresAt: timestamp("expires_at").notNull(),
  createdAt: timestamp("created_at").defaultNow().notNull(),
  updatedAt: timestamp("updated_at").defaultNow().notNull(),
});

// ─── Custom Tables ───────────────────────────────────────────────────

export const backgroundAssessment = pgTable(
  "background_assessment",
  {
    id: text("id").primaryKey(),
    userId: text("user_id")
      .notNull()
      .references(() => user.id, { onDelete: "cascade" })
      .unique(),
    devExperience: text("dev_experience").notNull(),
    pythonProficiency: text("python_proficiency").notNull(),
    roboticsBackground: text("robotics_background").notNull(),
    rosExposure: text("ros_exposure").notNull(),
    learningGoals: jsonb("learning_goals").notNull().$type<string[]>(),
    computedLevel: text("computed_level").notNull(),
    completedAt: timestamp("completed_at").defaultNow().notNull(),
    updatedAt: timestamp("updated_at").defaultNow().notNull(),
  },
  (table) => [
    index("idx_assessment_userId").on(table.userId),
  ]
);

export const chapterProgress = pgTable(
  "chapter_progress",
  {
    id: text("id").primaryKey(),
    userId: text("user_id")
      .notNull()
      .references(() => user.id, { onDelete: "cascade" }),
    chapterId: text("chapter_id").notNull(),
    status: text("status").notNull().default("not_started"),
    startedAt: timestamp("started_at"),
    completedAt: timestamp("completed_at"),
    lastAccessedAt: timestamp("last_accessed_at").defaultNow().notNull(),
  },
  (table) => [
    uniqueIndex("idx_progress_userId_chapter").on(
      table.userId,
      table.chapterId
    ),
    index("idx_progress_userId").on(table.userId),
    index("idx_progress_lastAccessed").on(table.userId, table.lastAccessedAt),
  ]
);

export const learningPath = pgTable(
  "learning_path",
  {
    id: text("id").primaryKey(),
    userId: text("user_id")
      .notNull()
      .references(() => user.id, { onDelete: "cascade" })
      .unique(),
    recommendedChapters: jsonb("recommended_chapters").notNull().$type<
      Array<{
        chapterId: string;
        chapterTitle: string;
        moduleId: number;
        priority: "required" | "recommended" | "optional";
      }>
    >(),
    priorityModules: jsonb("priority_modules").notNull().$type<number[]>(),
    startingChapter: text("starting_chapter").notNull(),
    generatedAt: timestamp("generated_at").defaultNow().notNull(),
    assessmentVersion: integer("assessment_version").notNull().default(1),
  },
  (table) => [
    index("idx_path_userId").on(table.userId),
  ]
);
