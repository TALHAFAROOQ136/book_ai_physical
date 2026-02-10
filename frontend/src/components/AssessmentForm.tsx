import React, { useState } from "react";

type AssessmentData = {
  devExperience: string;
  pythonProficiency: string;
  roboticsBackground: string;
  rosExposure: string;
  learningGoals: string[];
};

type Props = {
  initialData?: Partial<AssessmentData>;
  onSubmit: (data: AssessmentData) => Promise<void>;
  isSubmitting?: boolean;
};

const GOALS = [
  { value: "simulation", label: "Simulation & Digital Twins" },
  { value: "perception", label: "Perception & Computer Vision" },
  { value: "navigation", label: "Navigation & Path Planning" },
  { value: "voice_control", label: "Voice Control & NLU" },
  { value: "full_stack_robotics", label: "Full-Stack Robotics" },
];

export default function AssessmentForm({ initialData, onSubmit, isSubmitting }: Props) {
  const [form, setForm] = useState<AssessmentData>({
    devExperience: initialData?.devExperience || "",
    pythonProficiency: initialData?.pythonProficiency || "",
    roboticsBackground: initialData?.roboticsBackground || "",
    rosExposure: initialData?.rosExposure || "",
    learningGoals: initialData?.learningGoals || [],
  });
  const [error, setError] = useState("");

  const handleRadio = (field: keyof AssessmentData, value: string) => {
    setForm((prev) => ({ ...prev, [field]: value }));
  };

  const handleGoalToggle = (goal: string) => {
    setForm((prev) => ({
      ...prev,
      learningGoals: prev.learningGoals.includes(goal)
        ? prev.learningGoals.filter((g) => g !== goal)
        : [...prev.learningGoals, goal],
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    if (!form.devExperience || !form.pythonProficiency || !form.roboticsBackground || !form.rosExposure) {
      setError("Please answer all questions.");
      return;
    }
    if (form.learningGoals.length === 0) {
      setError("Please select at least one learning goal.");
      return;
    }
    await onSubmit(form);
  };

  const radioGroup = (
    label: string,
    field: keyof AssessmentData,
    options: { value: string; label: string }[]
  ) => (
    <fieldset style={{ marginBottom: "1.5rem", border: "none", padding: 0 }} aria-required="true">
      <legend style={{ fontWeight: 600, marginBottom: "0.5rem", fontSize: "1rem" }}>{label}</legend>
      {options.map((opt) => (
        <label
          key={opt.value}
          style={{ display: "block", padding: "0.25rem 0", cursor: "pointer" }}
        >
          <input
            type="radio"
            name={field}
            value={opt.value}
            checked={form[field] === opt.value}
            onChange={() => handleRadio(field, opt.value)}
            style={{ marginRight: "0.5rem" }}
          />
          {opt.label}
        </label>
      ))}
    </fieldset>
  );

  return (
    <form onSubmit={handleSubmit}>
      {error && (
        <div
          role="alert"
          style={{
            padding: "0.75rem",
            background: "#fef2f2",
            border: "1px solid #fca5a5",
            borderRadius: 6,
            color: "#dc2626",
            marginBottom: "1rem",
          }}
        >
          {error}
        </div>
      )}

      {radioGroup("1. What is your software development experience level?", "devExperience", [
        { value: "beginner", label: "Beginner — New to programming" },
        { value: "intermediate", label: "Intermediate — Comfortable with at least one language" },
        { value: "advanced", label: "Advanced — Professional developer" },
      ])}

      {radioGroup("2. How would you rate your Python proficiency?", "pythonProficiency", [
        { value: "none", label: "None — Never used Python" },
        { value: "basic", label: "Basic — Simple scripts and tutorials" },
        { value: "proficient", label: "Proficient — Build projects in Python" },
        { value: "expert", label: "Expert — Python is my primary language" },
      ])}

      {radioGroup("3. What is your hardware/robotics background?", "roboticsBackground", [
        { value: "none", label: "None — No robotics experience" },
        { value: "hobbyist", label: "Hobbyist — Arduino, Raspberry Pi, hobby kits" },
        { value: "professional", label: "Professional — Industry or research robotics" },
      ])}

      {radioGroup("4. What is your prior ROS exposure?", "rosExposure", [
        { value: "none", label: "None — Never used ROS" },
        { value: "ros1", label: "ROS 1 — Familiar with ROS 1" },
        { value: "ros2", label: "ROS 2 — Worked with ROS 2" },
      ])}

      <fieldset style={{ marginBottom: "1.5rem", border: "none", padding: 0 }} aria-required="true">
        <legend style={{ fontWeight: 600, marginBottom: "0.5rem", fontSize: "1rem" }}>
          5. What are your learning goals? (Select all that apply)
        </legend>
        {GOALS.map((goal) => (
          <label
            key={goal.value}
            style={{ display: "block", padding: "0.25rem 0", cursor: "pointer" }}
          >
            <input
              type="checkbox"
              checked={form.learningGoals.includes(goal.value)}
              onChange={() => handleGoalToggle(goal.value)}
              style={{ marginRight: "0.5rem" }}
            />
            {goal.label}
          </label>
        ))}
      </fieldset>

      <button
        type="submit"
        disabled={isSubmitting}
        style={{
          padding: "0.75rem 2rem",
          background: "#2e8555",
          color: "#fff",
          border: "none",
          borderRadius: 6,
          fontSize: "1rem",
          cursor: isSubmitting ? "not-allowed" : "pointer",
          opacity: isSubmitting ? 0.7 : 1,
        }}
      >
        {isSubmitting ? "Saving..." : "Submit Assessment"}
      </button>
    </form>
  );
}
