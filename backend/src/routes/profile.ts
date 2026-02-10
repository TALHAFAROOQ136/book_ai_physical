import { Hono } from "hono";
import { eq } from "drizzle-orm";
import { db } from "../db/index.js";
import { user, backgroundAssessment, chapterProgress, learningPath } from "../db/schema.js";
import { requireAuth, type AuthUser } from "../middleware/auth.js";

export const profileRoutes = new Hono();

// GET /api/profile — Get full user profile
profileRoutes.get("/", requireAuth, async (c) => {
  const authUser = c.get("user") as AuthUser;

  const [userRecord] = await db
    .select()
    .from(user)
    .where(eq(user.id, authUser.id))
    .limit(1);

  if (!userRecord) {
    return c.json({ message: "User not found", code: "NOT_FOUND" }, 404);
  }

  const [assessment] = await db
    .select()
    .from(backgroundAssessment)
    .where(eq(backgroundAssessment.userId, authUser.id))
    .limit(1);

  const progress = await db
    .select()
    .from(chapterProgress)
    .where(eq(chapterProgress.userId, authUser.id));

  const completedCount = progress.filter((p) => p.status === "completed").length;
  const inProgressCount = progress.filter((p) => p.status === "in_progress").length;

  const [path] = await db
    .select()
    .from(learningPath)
    .where(eq(learningPath.userId, authUser.id))
    .limit(1);

  return c.json({
    user: {
      id: userRecord.id,
      email: userRecord.email,
      name: userRecord.name,
      displayName: userRecord.displayName,
      createdAt: userRecord.createdAt,
    },
    assessment: assessment || null,
    progressSummary: {
      totalChapters: progress.length,
      completedCount,
      inProgressCount,
      overallPercentage: progress.length > 0
        ? Math.round((completedCount / progress.length) * 100)
        : 0,
    },
    learningPath: path || null,
  });
});

// PATCH /api/profile — Update display name
profileRoutes.patch("/", requireAuth, async (c) => {
  const authUser = c.get("user") as AuthUser;
  const body = await c.req.json();

  const updates: Record<string, unknown> = { updatedAt: new Date() };
  if (body.displayName && typeof body.displayName === "string") {
    updates.displayName = body.displayName.trim();
  }
  if (body.name && typeof body.name === "string") {
    updates.name = body.name.trim();
  }

  await db.update(user).set(updates).where(eq(user.id, authUser.id));

  return c.json({ message: "Profile updated" });
});

// DELETE /api/profile — Soft-delete account (sets deletedAt, terminates session)
profileRoutes.delete("/", requireAuth, async (c) => {
  const authUser = c.get("user") as AuthUser;

  await db
    .update(user)
    .set({ deletedAt: new Date(), updatedAt: new Date() })
    .where(eq(user.id, authUser.id));

  return c.json({ message: "Account scheduled for deletion. Data will be purged after 30 days." });
});
