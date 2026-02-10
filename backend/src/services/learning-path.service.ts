import { eq } from "drizzle-orm";
import { db } from "../db/index.js";
import { learningPath, chapterProgress } from "../db/schema.js";

// Rule table: learning goals → priority modules (Module 1 always prerequisite)
const GOAL_MODULE_MAP: Record<string, number[]> = {
  simulation: [2, 3],
  perception: [3, 1],
  navigation: [3, 1],
  voice_control: [4, 1],
  full_stack_robotics: [1, 2, 3, 4],
};

// All chapters organized by module
const MODULE_CHAPTERS: Record<
  number,
  Array<{ chapterId: string; chapterTitle: string }>
> = {
  0: [{ chapterId: "intro", chapterTitle: "Physical AI & Humanoid Robotics" }],
  1: [{ chapterId: "module1/intro-to-ros2", chapterTitle: "Introduction to ROS 2" }],
  2: [{ chapterId: "module2/intro-to-simulation", chapterTitle: "Introduction to Simulation" }],
  3: [{ chapterId: "module3/intro-to-isaac", chapterTitle: "Introduction to NVIDIA Isaac" }],
  4: [{ chapterId: "module4/intro-to-vla", chapterTitle: "Introduction to Vision-Language-Action" }],
};

export async function generateLearningPath(
  userId: string,
  learningGoals: string[]
): Promise<void> {
  // Compute priority modules from goals
  const prioritySet = new Set<number>();
  prioritySet.add(1); // Module 1 always prerequisite

  for (const goal of learningGoals) {
    const modules = GOAL_MODULE_MAP[goal];
    if (modules) modules.forEach((m) => prioritySet.add(m));
  }

  const priorityModules = Array.from(prioritySet).sort();

  // Build recommended chapters in module order
  type ChapterRec = {
    chapterId: string;
    chapterTitle: string;
    moduleId: number;
    priority: "required" | "recommended" | "optional";
  };

  const chapters: ChapterRec[] = [];

  // Add intro as required
  chapters.push({
    chapterId: "intro",
    chapterTitle: "Physical AI & Humanoid Robotics",
    moduleId: 0,
    priority: "required",
  });

  // Add chapters by module
  for (let moduleId = 1; moduleId <= 4; moduleId++) {
    const moduleChapters = MODULE_CHAPTERS[moduleId] || [];
    const isPriority = priorityModules.includes(moduleId);

    for (const ch of moduleChapters) {
      chapters.push({
        ...ch,
        moduleId,
        priority: moduleId === 1 ? "required" : isPriority ? "recommended" : "optional",
      });
    }
  }

  // Determine starting chapter
  const startingChapter =
    priorityModules.length > 0
      ? MODULE_CHAPTERS[1]?.[0]?.chapterId || "intro"
      : "intro";

  // Upsert learning path
  const existing = await db
    .select()
    .from(learningPath)
    .where(eq(learningPath.userId, userId))
    .limit(1);

  const now = new Date();

  if (existing.length > 0) {
    await db
      .update(learningPath)
      .set({
        recommendedChapters: chapters,
        priorityModules,
        startingChapter,
        generatedAt: now,
        assessmentVersion: (existing[0].assessmentVersion || 0) + 1,
      })
      .where(eq(learningPath.userId, userId));
  } else {
    await db.insert(learningPath).values({
      id: crypto.randomUUID(),
      userId,
      recommendedChapters: chapters,
      priorityModules,
      startingChapter,
      generatedAt: now,
      assessmentVersion: 1,
    });
  }
}

export async function getLearningPath(userId: string) {
  const [path] = await db
    .select()
    .from(learningPath)
    .where(eq(learningPath.userId, userId))
    .limit(1);

  if (!path) return null;

  // Merge with progress data
  const progress = await db
    .select()
    .from(chapterProgress)
    .where(eq(chapterProgress.userId, userId));

  const progressMap = new Map(progress.map((p) => [p.chapterId, p.status]));

  const chaptersWithProgress = (
    path.recommendedChapters as Array<{
      chapterId: string;
      chapterTitle: string;
      moduleId: number;
      priority: string;
    }>
  ).map((ch) => ({
    ...ch,
    status: progressMap.get(ch.chapterId) || "not_started",
  }));

  return {
    ...path,
    recommendedChapters: chaptersWithProgress,
  };
}
