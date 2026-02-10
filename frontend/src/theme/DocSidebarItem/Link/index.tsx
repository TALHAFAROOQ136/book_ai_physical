import React from "react";
import LinkOrig from "@theme-original/DocSidebarItem/Link";
import { useAuthContext } from "../../../components/AuthProvider";

/**
 * Swizzled sidebar link that shows completion indicators:
 * - Checkmark for completed chapters
 * - Dot for in-progress chapters
 */
export default function LinkWrapper(props: any) {
  const { isAuthenticated, chapterStatuses } = useAuthContext();

  // Extract chapter ID from the sidebar item's href
  const href: string = props.item?.href || "";
  const chapterId = href.replace(/^\/chapters\//, "").replace(/\/$/, "");
  const status = isAuthenticated && chapterId ? chapterStatuses[chapterId] : undefined;

  return (
    <div style={{ position: "relative", display: "flex", alignItems: "center" }}>
      <LinkOrig {...props} />
      {status === "completed" && (
        <span
          title="Completed"
          aria-label="Chapter completed"
          style={{
            position: "absolute",
            right: 8,
            color: "#16a34a",
            fontSize: "0.875rem",
            fontWeight: 700,
            lineHeight: 1,
            pointerEvents: "none",
          }}
        >
          &#x2713;
        </span>
      )}
      {status === "in_progress" && (
        <span
          title="In progress"
          aria-label="Chapter in progress"
          style={{
            position: "absolute",
            right: 10,
            width: 8,
            height: 8,
            borderRadius: "50%",
            background: "#d97706",
            pointerEvents: "none",
          }}
        />
      )}
    </div>
  );
}
