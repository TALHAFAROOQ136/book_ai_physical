import { Hono } from "hono";
import { requireAuth, type AuthUser } from "../middleware/auth.js";
import {
  getProgressForUser,
  getChapterProgress,
  updateChapterProgress,
  getContinueChapter,
} from "../services/progress.service.js";

export const progressRoutes = new Hono();

// GET /api/progress — Get all chapter progress
progressRoutes.get("/", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const result = await getProgressForUser(user.id);
  return c.json(result);
});

// GET /api/progress/continue — Get "continue where you left off"
progressRoutes.get("/continue", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const result = await getContinueChapter(user.id);
  if (!result) {
    return c.json(
      { message: "No chapters started yet", code: "NOT_FOUND" },
      404
    );
  }
  return c.json(result);
});

// GET /api/progress/:chapterId — Get specific chapter progress
progressRoutes.get("/:chapterId{.+}", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const chapterId = c.req.param("chapterId");
  const result = await getChapterProgress(user.id, chapterId);
  if (!result) {
    return c.json(
      { message: "No progress record", code: "NOT_FOUND" },
      404
    );
  }
  return c.json(result);
});

// PATCH /api/progress/:chapterId — Update chapter status
progressRoutes.patch("/:chapterId{.+}", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const chapterId = c.req.param("chapterId");
  const body = await c.req.json();

  if (!["in_progress", "completed"].includes(body.status)) {
    return c.json(
      { message: "Status must be 'in_progress' or 'completed'", code: "VALIDATION_ERROR" },
      422
    );
  }

  const result = await updateChapterProgress(user.id, chapterId, body.status);
  return c.json(result);
});
