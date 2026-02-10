import React, { useState, useEffect } from "react";
import { useAuthContext } from "./AuthProvider";

type Props = {
  level: "beginner" | "intermediate" | "advanced";
  summary: string;
  children: React.ReactNode;
  defaultOpen?: boolean;
};

/**
 * A <details> wrapper that adjusts its default open/closed state
 * based on the user's computed assessment level.
 *
 * Usage in MDX:
 * <PersonalizedDetails level="beginner" summary="New to Python?">
 *   Expanded content for beginners...
 * </PersonalizedDetails>
 *
 * Behavior:
 * - Beginner user + level="beginner" → expanded by default
 * - Advanced user + level="beginner" → collapsed by default
 * - Intermediate user → uses defaultOpen prop or standard state
 * - Manual toggle always works (FR-015)
 */
export default function PersonalizedDetails({
  level,
  summary,
  children,
  defaultOpen,
}: Props) {
  const { assessmentLevel } = useAuthContext();
  const [isOpen, setIsOpen] = useState(false);
  const [initialized, setInitialized] = useState(false);

  useEffect(() => {
    if (initialized) return;

    let shouldOpen = defaultOpen ?? false;

    if (assessmentLevel === "beginner" && level === "beginner") {
      shouldOpen = true;
    } else if (assessmentLevel === "advanced" && level === "beginner") {
      shouldOpen = false;
    } else if (assessmentLevel === "intermediate") {
      shouldOpen = defaultOpen ?? true; // Show by default for intermediate
    }

    setIsOpen(shouldOpen);
    setInitialized(true);
  }, [assessmentLevel, level, defaultOpen, initialized]);

  return (
    <details
      open={isOpen || undefined}
      data-level={level}
      onToggle={(e) => setIsOpen((e.target as HTMLDetailsElement).open)}
    >
      <summary style={{ cursor: "pointer", fontWeight: 600 }}>{summary}</summary>
      <div style={{ paddingTop: "0.5rem" }}>{children}</div>
    </details>
  );
}
