import { eq, and, desc } from "drizzle-orm";
import { db } from "../db/index.js";
import { chapterProgress } from "../db/schema.js";

// Chapter-to-module mapping (matches constitution course structure)
const CHAPTER_MODULE_MAP: Record<string, { moduleId: number; moduleName: string }> = {
  intro: { moduleId: 0, moduleName: "Introduction" },
  "module1/intro-to-ros2": { moduleId: 1, moduleName: "Module 1: ROS 2" },
  "module2/intro-to-simulation": { moduleId: 2, moduleName: "Module 2: Digital Twin" },
  "module3/intro-to-isaac": { moduleId: 3, moduleName: "Module 3: NVIDIA Isaac" },
  "module4/intro-to-vla": { moduleId: 4, moduleName: "Module 4: VLA" },
};

const ALL_CHAPTERS = Object.keys(CHAPTER_MODULE_MAP);

type ModuleProgress = {
  moduleId: number;
  moduleName: string;
  totalChapters: number;
  completedChapters: number;
  percentage: number;
};

export async function getProgressForUser(userId: string) {
  const records = await db
    .select()
    .from(chapterProgress)
    .where(eq(chapterProgress.userId, userId));

  const progressMap = new Map(records.map((r) => [r.chapterId, r]));

  // Build per-module progress
  const moduleMap = new Map<number, { name: string; total: number; completed: number }>();
  for (const [chapterId, info] of Object.entries(CHAPTER_MODULE_MAP)) {
    const entry = moduleMap.get(info.moduleId) || {
      name: info.moduleName,
      total: 0,
      completed: 0,
    };
    entry.total++;
    const progress = progressMap.get(chapterId);
    if (progress?.status === "completed") entry.completed++;
    moduleMap.set(info.moduleId, entry);
  }

  const moduleProgress: ModuleProgress[] = Array.from(moduleMap.entries()).map(
    ([moduleId, data]) => ({
      moduleId,
      moduleName: data.name,
      totalChapters: data.total,
      completedChapters: data.completed,
      percentage: data.total > 0 ? Math.round((data.completed / data.total) * 100) : 0,
    })
  );

  const completedCount = records.filter((r) => r.status === "completed").length;
  const inProgressCount = records.filter((r) => r.status === "in_progress").length;

  return {
    chapters: records,
    summary: {
      totalChapters: ALL_CHAPTERS.length,
      completedCount,
      inProgressCount,
      overallPercentage:
        ALL_CHAPTERS.length > 0
          ? Math.round((completedCount / ALL_CHAPTERS.length) * 100)
          : 0,
      moduleProgress,
    },
  };
}

export async function getChapterProgress(userId: string, chapterId: string) {
  const [record] = await db
    .select()
    .from(chapterProgress)
    .where(
      and(
        eq(chapterProgress.userId, userId),
        eq(chapterProgress.chapterId, chapterId)
      )
    )
    .limit(1);
  return record || null;
}

export async function updateChapterProgress(
  userId: string,
  chapterId: string,
  status: "in_progress" | "completed"
) {
  const existing = await getChapterProgress(userId, chapterId);
  const now = new Date();

  if (existing) {
    const updates: Record<string, unknown> = {
      status,
      lastAccessedAt: now,
    };
    if (status === "completed" && !existing.completedAt) {
      updates.completedAt = now;
    }
    await db
      .update(chapterProgress)
      .set(updates)
      .where(eq(chapterProgress.id, existing.id));
    return { ...existing, ...updates };
  }

  const record = {
    id: crypto.randomUUID(),
    userId,
    chapterId,
    status,
    startedAt: now,
    completedAt: status === "completed" ? now : null,
    lastAccessedAt: now,
  };
  await db.insert(chapterProgress).values(record);
  return record;
}

export async function getContinueChapter(userId: string) {
  const [record] = await db
    .select()
    .from(chapterProgress)
    .where(
      and(
        eq(chapterProgress.userId, userId),
        eq(chapterProgress.status, "in_progress")
      )
    )
    .orderBy(desc(chapterProgress.lastAccessedAt))
    .limit(1);

  if (!record) return null;

  const info = CHAPTER_MODULE_MAP[record.chapterId];
  return {
    chapterId: record.chapterId,
    chapterTitle: info?.moduleName || record.chapterId,
    lastAccessedAt: record.lastAccessedAt,
  };
}
