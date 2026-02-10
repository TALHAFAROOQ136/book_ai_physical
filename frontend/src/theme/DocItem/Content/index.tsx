import React, { useEffect, useState } from "react";
import ContentOrig from "@theme-original/DocItem/Content";
import { useAuthContext } from "../../../components/AuthProvider";

const API_URL = "http://localhost:3001";

function getChapterIdFromPath(): string | null {
  if (typeof window === "undefined") return null;
  const path = window.location.pathname.replace(/^\/chapters\//, "").replace(/\/$/, "");
  return path || null;
}

export default function ContentWrapper(props: any) {
  const { isAuthenticated, assessmentLevel } = useAuthContext();
  const [completed, setCompleted] = useState(false);
  const chapterId = getChapterIdFromPath();

  // Track page access as in_progress
  useEffect(() => {
    if (!isAuthenticated || !chapterId) return;
    fetch(`${API_URL}/api/progress/${encodeURIComponent(chapterId)}`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      credentials: "include",
      body: JSON.stringify({ status: "in_progress" }),
    }).catch(() => {});
  }, [isAuthenticated, chapterId]);

  // Apply content personalization: toggle details elements
  useEffect(() => {
    if (typeof document === "undefined") return;
    const details = document.querySelectorAll("details[data-level]");
    details.forEach((el) => {
      const level = el.getAttribute("data-level");
      if (assessmentLevel === "beginner" && level === "beginner") {
        el.setAttribute("open", "");
      } else if (assessmentLevel === "advanced" && level === "beginner") {
        el.removeAttribute("open");
      }
      // Intermediate: leave as authored
    });
  }, [assessmentLevel]);

  const handleMarkComplete = async () => {
    if (!chapterId) return;
    try {
      await fetch(`${API_URL}/api/progress/${encodeURIComponent(chapterId)}`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ status: "completed" }),
      });
      setCompleted(true);
    } catch {
      // Silently fail
    }
  };

  return (
    <>
      <ContentOrig {...props} />
      {isAuthenticated && chapterId && (
        <div
          style={{
            marginTop: "3rem",
            padding: "1.5rem",
            background: completed ? "#f0fdf4" : "var(--ifm-background-surface-color)",
            border: `1px solid ${completed ? "#86efac" : "var(--ifm-color-emphasis-200)"}`,
            borderRadius: 8,
            textAlign: "center",
          }}
        >
          {completed ? (
            <p style={{ margin: 0, fontWeight: 600, color: "#16a34a" }}>
              Chapter completed! Great work.
            </p>
          ) : (
            <>
              <p style={{ margin: "0 0 0.75rem" }}>
                Finished this chapter?
              </p>
              <button
                onClick={handleMarkComplete}
                style={{
                  padding: "0.5rem 2rem",
                  background: "#2e8555",
                  color: "#fff",
                  border: "none",
                  borderRadius: 6,
                  fontSize: "1rem",
                  cursor: "pointer",
                }}
              >
                Mark as Complete
              </button>
            </>
          )}
        </div>
      )}
    </>
  );
}
