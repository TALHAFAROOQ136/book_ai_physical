import React, { useEffect, useState } from "react";
import ProtectedRoute from "../components/ProtectedRoute";
import ProgressBar from "../components/ProgressBar";
import LearningPathView from "../components/LearningPathView";

const API_URL = "http://localhost:3001";

type ContinueData = {
  chapterId: string;
  chapterTitle: string;
  lastAccessedAt: string;
};

type ProgressData = {
  chapters: Array<{ chapterId: string; status: string; lastAccessedAt: string }>;
  summary: {
    totalChapters: number;
    completedCount: number;
    inProgressCount: number;
    overallPercentage: number;
    moduleProgress: Array<{
      moduleId: number;
      moduleName: string;
      totalChapters: number;
      completedChapters: number;
      percentage: number;
    }>;
  };
};

type LearningPathData = {
  recommendedChapters: Array<{
    chapterId: string;
    chapterTitle: string;
    moduleId: number;
    priority: string;
    status?: string;
  }>;
  priorityModules: number[];
  startingChapter: string;
} | null;

export default function DashboardPage() {
  const [continueData, setContinueData] = useState<ContinueData | null>(null);
  const [progress, setProgress] = useState<ProgressData | null>(null);
  const [learningPath, setLearningPath] = useState<LearningPathData>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    (async () => {
      try {
        const [continueRes, progressRes, pathRes] = await Promise.all([
          fetch(`${API_URL}/api/progress/continue`, { credentials: "include" }),
          fetch(`${API_URL}/api/progress`, { credentials: "include" }),
          fetch(`${API_URL}/api/learning-path`, { credentials: "include" }),
        ]);

        if (continueRes.ok) setContinueData(await continueRes.json());
        if (progressRes.ok) setProgress(await progressRes.json());
        if (pathRes.ok) setLearningPath(await pathRes.json());
      } catch {
        // Silently fail individual requests
      } finally {
        setIsLoading(false);
      }
    })();
  }, []);

  return (
    <ProtectedRoute title="Dashboard" description="Your learning progress">
      <div style={{ maxWidth: 800, margin: "2rem auto", padding: "0 1rem" }}>
        <h1>Your Dashboard</h1>

        {isLoading ? (
          <p>Loading your progress...</p>
        ) : (
          <>
            {/* Continue Where You Left Off */}
            {continueData && (
              <div
                style={{
                  padding: "1.5rem",
                  background: "#eff6ff",
                  border: "1px solid #93c5fd",
                  borderRadius: 8,
                  marginBottom: "2rem",
                }}
              >
                <h3 style={{ marginTop: 0 }}>Continue where you left off</h3>
                <p>
                  <strong>{continueData.chapterTitle}</strong>
                  <br />
                  <small>
                    Last accessed:{" "}
                    {new Date(continueData.lastAccessedAt).toLocaleDateString()}
                  </small>
                </p>
                <a
                  href={`/chapters/${continueData.chapterId}`}
                  style={{
                    display: "inline-block",
                    padding: "0.5rem 1.5rem",
                    background: "#2e8555",
                    color: "#fff",
                    borderRadius: 6,
                    textDecoration: "none",
                  }}
                >
                  Continue Reading
                </a>
              </div>
            )}

            {/* Progress Overview */}
            {progress && (
              <div style={{ marginBottom: "2rem" }}>
                <h2>Progress Overview</h2>
                <ProgressBar
                  overallPercentage={progress.summary.overallPercentage}
                  moduleProgress={progress.summary.moduleProgress}
                />
                <p style={{ fontSize: "0.875rem", color: "var(--ifm-color-emphasis-600)" }}>
                  {progress.summary.completedCount} of{" "}
                  {progress.summary.totalChapters} chapters completed
                  {progress.summary.inProgressCount > 0 &&
                    ` | ${progress.summary.inProgressCount} in progress`}
                </p>
              </div>
            )}

            {/* Learning Path */}
            {learningPath ? (
              <div style={{ marginBottom: "2rem" }}>
                <h2>Your Learning Path</h2>
                <LearningPathView
                  chapters={learningPath.recommendedChapters}
                  priorityModules={learningPath.priorityModules}
                />
              </div>
            ) : (
              <div
                style={{
                  padding: "1rem",
                  background: "var(--ifm-background-surface-color)",
                  border: "1px solid var(--ifm-color-emphasis-200)",
                  borderRadius: 8,
                }}
              >
                <p>
                  <a href="/assessment">Complete your background assessment</a>{" "}
                  to get a personalized learning path.
                </p>
              </div>
            )}
          </>
        )}
      </div>
    </ProtectedRoute>
  );
}
