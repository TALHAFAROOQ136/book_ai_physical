import { Hono } from "hono";
import { requireAuth, type AuthUser } from "../middleware/auth.js";
import {
  getAssessment,
  upsertAssessment,
  validateAssessmentInput,
  computeLevel,
  generateStartingRecommendation,
} from "../services/assessment.service.js";
import { generateLearningPath } from "../services/learning-path.service.js";

export const assessmentRoutes = new Hono();

// GET /api/assessment — Get current user's assessment
assessmentRoutes.get("/", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const assessment = await getAssessment(user.id);

  if (!assessment) {
    return c.json({ message: "No assessment completed yet", code: "NOT_FOUND" }, 404);
  }

  return c.json(assessment);
});

// POST /api/assessment — Create or update assessment
assessmentRoutes.post("/", requireAuth, async (c) => {
  const user = c.get("user") as AuthUser;
  const body = await c.req.json();

  const validation = validateAssessmentInput(body);
  if (!validation.valid) {
    return c.json({ message: validation.error, code: "VALIDATION_ERROR" }, 422);
  }

  const assessment = await upsertAssessment(user.id, validation.data);
  const level = computeLevel(validation.data);
  const recommendation = generateStartingRecommendation(level);

  // Regenerate learning path when assessment changes
  try {
    await generateLearningPath(user.id, validation.data.learningGoals);
  } catch {
    // Non-blocking — path generation is secondary
  }

  return c.json({ assessment, recommendation });
});
