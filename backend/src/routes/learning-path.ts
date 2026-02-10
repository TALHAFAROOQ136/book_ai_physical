import { Hono } from "hono";
import { requireAuth, type AuthUser } from "../middleware/auth.js";
import { getLearningPath, generateLearningPath } from "../services/learning-path.service.js";
import { getAssessment } from "../services/assessment.service.js";

export const learningPathRoutes = new Hono();

// GET /api/learning-path — Get personalized learning path
learningPathRoutes.get("/", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const path = await getLearningPath(user.id);

  if (!path) {
    return c.json(
      { message: "No assessment completed. Complete your assessment to get a personalized path.", code: "NOT_FOUND" },
      404
    );
  }

  return c.json(path);
});

// POST /api/learning-path — Regenerate learning path
learningPathRoutes.post("/", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;

  const assessment = await getAssessment(user.id);
  if (!assessment) {
    return c.json(
      { message: "No assessment found. Complete your assessment first.", code: "NOT_FOUND" },
      404
    );
  }

  await generateLearningPath(user.id, assessment.learningGoals as string[]);
  const path = await getLearningPath(user.id);

  return c.json(path);
});
