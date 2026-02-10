import React from "react";

type Chapter = {
  chapterId: string;
  chapterTitle: string;
  moduleId: number;
  priority: string;
  status?: string;
};

type Props = {
  chapters: Chapter[];
  priorityModules: number[];
};

const MODULE_NAMES: Record<number, string> = {
  0: "Introduction",
  1: "ROS 2 Fundamentals",
  2: "Simulation",
  3: "NVIDIA Isaac",
  4: "Vision-Language-Action",
};

const PRIORITY_LABELS: Record<string, { text: string; color: string }> = {
  required: { text: "Required", color: "#dc2626" },
  recommended: { text: "Recommended", color: "#2563eb" },
  optional: { text: "Optional", color: "#6b7280" },
};

const STATUS_LABELS: Record<string, { text: string; color: string }> = {
  completed: { text: "Done", color: "#16a34a" },
  in_progress: { text: "In Progress", color: "#d97706" },
  not_started: { text: "", color: "transparent" },
};

export default function LearningPathView({ chapters, priorityModules }: Props) {
  // Group chapters by module
  const grouped = new Map<number, Chapter[]>();
  for (const ch of chapters) {
    const list = grouped.get(ch.moduleId) || [];
    list.push(ch);
    grouped.set(ch.moduleId, list);
  }

  const sortedModuleIds = Array.from(grouped.keys()).sort((a, b) => a - b);

  return (
    <div style={{ display: "flex", flexDirection: "column", gap: "1.5rem" }}>
      {sortedModuleIds.map((moduleId) => {
        const moduleChapters = grouped.get(moduleId) || [];
        const isPriority = priorityModules.includes(moduleId);

        return (
          <div key={moduleId}>
            <h4
              style={{
                margin: "0 0 0.5rem",
                display: "flex",
                alignItems: "center",
                gap: "0.5rem",
              }}
            >
              {MODULE_NAMES[moduleId] || `Module ${moduleId}`}
              {isPriority && (
                <span
                  style={{
                    fontSize: "0.75rem",
                    padding: "0.125rem 0.5rem",
                    background: "#dbeafe",
                    color: "#1d4ed8",
                    borderRadius: 4,
                  }}
                >
                  Priority
                </span>
              )}
            </h4>
            <div style={{ display: "flex", flexDirection: "column", gap: "0.375rem" }}>
              {moduleChapters.map((ch) => {
                const priority = PRIORITY_LABELS[ch.priority] || PRIORITY_LABELS.optional;
                const status = STATUS_LABELS[ch.status || "not_started"] || STATUS_LABELS.not_started;

                return (
                  <a
                    key={ch.chapterId}
                    href={`/chapters/${ch.chapterId}`}
                    style={{
                      display: "flex",
                      justifyContent: "space-between",
                      alignItems: "center",
                      padding: "0.625rem 1rem",
                      border: "1px solid var(--ifm-color-emphasis-200)",
                      borderRadius: 6,
                      textDecoration: "none",
                      color: "inherit",
                    }}
                  >
                    <span style={{ display: "flex", alignItems: "center", gap: "0.5rem" }}>
                      <strong>{ch.chapterTitle}</strong>
                      <small style={{ color: priority.color }}>({priority.text})</small>
                    </span>
                    {status.text && (
                      <span
                        style={{
                          fontSize: "0.8125rem",
                          fontWeight: 600,
                          color: status.color,
                        }}
                      >
                        {status.text}
                      </span>
                    )}
                  </a>
                );
              })}
            </div>
          </div>
        );
      })}
    </div>
  );
}
