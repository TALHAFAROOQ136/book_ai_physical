import { eq } from "drizzle-orm";
import { db } from "../db/index.js";
import { backgroundAssessment } from "../db/schema.js";

type AssessmentInput = {
  devExperience: "beginner" | "intermediate" | "advanced";
  pythonProficiency: "none" | "basic" | "proficient" | "expert";
  roboticsBackground: "none" | "hobbyist" | "professional";
  rosExposure: "none" | "ros1" | "ros2";
  learningGoals: string[];
};

type ComputedLevel = "beginner" | "intermediate" | "advanced";

const VALID_DEV_EXPERIENCE = ["beginner", "intermediate", "advanced"];
const VALID_PYTHON = ["none", "basic", "proficient", "expert"];
const VALID_ROBOTICS = ["none", "hobbyist", "professional"];
const VALID_ROS = ["none", "ros1", "ros2"];
const VALID_GOALS = [
  "simulation",
  "perception",
  "navigation",
  "voice_control",
  "full_stack_robotics",
];

export function validateAssessmentInput(
  input: unknown
): { valid: true; data: AssessmentInput } | { valid: false; error: string } {
  const body = input as Record<string, unknown>;

  if (!VALID_DEV_EXPERIENCE.includes(body.devExperience as string))
    return { valid: false, error: "Invalid devExperience value." };
  if (!VALID_PYTHON.includes(body.pythonProficiency as string))
    return { valid: false, error: "Invalid pythonProficiency value." };
  if (!VALID_ROBOTICS.includes(body.roboticsBackground as string))
    return { valid: false, error: "Invalid roboticsBackground value." };
  if (!VALID_ROS.includes(body.rosExposure as string))
    return { valid: false, error: "Invalid rosExposure value." };
  if (
    !Array.isArray(body.learningGoals) ||
    body.learningGoals.length === 0 ||
    !body.learningGoals.every((g: unknown) => VALID_GOALS.includes(g as string))
  )
    return { valid: false, error: "learningGoals must be a non-empty array of valid goals." };

  return { valid: true, data: body as unknown as AssessmentInput };
}

/**
 * Compute user level from assessment responses.
 * Count "advanced" indicators:
 *  - devExperience=advanced
 *  - pythonProficiency=expert
 *  - roboticsBackground=professional
 *  - rosExposure=ros2
 * 0-1 → beginner, 2 → intermediate, 3-4 → advanced
 */
export function computeLevel(input: AssessmentInput): ComputedLevel {
  let advancedCount = 0;
  if (input.devExperience === "advanced") advancedCount++;
  if (input.pythonProficiency === "expert") advancedCount++;
  if (input.roboticsBackground === "professional") advancedCount++;
  if (input.rosExposure === "ros2") advancedCount++;

  if (advancedCount <= 1) return "beginner";
  if (advancedCount === 2) return "intermediate";
  return "advanced";
}

/**
 * Generate a starting chapter recommendation based on computed level.
 */
export function generateStartingRecommendation(
  level: ComputedLevel
): { startingChapter: string; reason: string } {
  switch (level) {
    case "beginner":
      return {
        startingChapter: "intro",
        reason:
          "Start from the very beginning to build a solid foundation in Physical AI concepts.",
      };
    case "intermediate":
      return {
        startingChapter: "module1/intro-to-ros2",
        reason:
          "You have some background — jump into ROS 2 fundamentals to start building robot systems.",
      };
    case "advanced":
      return {
        startingChapter: "module2/intro-to-simulation",
        reason:
          "With strong fundamentals, skip to simulation and advanced topics that build on your experience.",
      };
  }
}

export async function getAssessment(userId: string) {
  const [result] = await db
    .select()
    .from(backgroundAssessment)
    .where(eq(backgroundAssessment.userId, userId))
    .limit(1);
  return result || null;
}

export async function upsertAssessment(
  userId: string,
  input: AssessmentInput
) {
  const level = computeLevel(input);
  const existing = await getAssessment(userId);
  const now = new Date();
  const id = existing?.id || crypto.randomUUID();

  if (existing) {
    await db
      .update(backgroundAssessment)
      .set({
        devExperience: input.devExperience,
        pythonProficiency: input.pythonProficiency,
        roboticsBackground: input.roboticsBackground,
        rosExposure: input.rosExposure,
        learningGoals: input.learningGoals,
        computedLevel: level,
        updatedAt: now,
      })
      .where(eq(backgroundAssessment.id, existing.id));
  } else {
    await db.insert(backgroundAssessment).values({
      id,
      userId,
      devExperience: input.devExperience,
      pythonProficiency: input.pythonProficiency,
      roboticsBackground: input.roboticsBackground,
      rosExposure: input.rosExposure,
      learningGoals: input.learningGoals,
      computedLevel: level,
      completedAt: now,
      updatedAt: now,
    });
  }

  return { id, ...input, computedLevel: level, userId };
}
