import React from "react";

type ModuleProgress = {
  moduleId: number;
  moduleName: string;
  totalChapters: number;
  completedChapters: number;
  percentage: number;
};

type Props = {
  overallPercentage: number;
  moduleProgress: ModuleProgress[];
};

function Bar({ percentage, label }: { percentage: number; label: string }) {
  return (
    <div style={{ marginBottom: "1rem" }}>
      <div
        style={{
          display: "flex",
          justifyContent: "space-between",
          marginBottom: 4,
          fontSize: "0.875rem",
        }}
      >
        <span>{label}</span>
        <span>{percentage}%</span>
      </div>
      <div
        style={{
          height: 8,
          background: "var(--ifm-color-emphasis-200)",
          borderRadius: 4,
          overflow: "hidden",
        }}
        role="progressbar"
        aria-valuenow={percentage}
        aria-valuemin={0}
        aria-valuemax={100}
        aria-label={`${label}: ${percentage}% complete`}
      >
        <div
          style={{
            height: "100%",
            width: `${percentage}%`,
            background: "#2e8555",
            borderRadius: 4,
            transition: "width 0.3s ease",
          }}
        />
      </div>
    </div>
  );
}

export default function ProgressBar({ overallPercentage, moduleProgress }: Props) {
  return (
    <div>
      <Bar percentage={overallPercentage} label="Overall Progress" />
      {moduleProgress.map((mod) => (
        <Bar
          key={mod.moduleId}
          percentage={mod.percentage}
          label={`${mod.moduleName} (${mod.completedChapters}/${mod.totalChapters})`}
        />
      ))}
    </div>
  );
}
