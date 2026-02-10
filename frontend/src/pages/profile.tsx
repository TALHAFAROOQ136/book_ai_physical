import React, { useEffect, useState } from "react";
import ProtectedRoute from "../components/ProtectedRoute";
import AssessmentForm from "../components/AssessmentForm";
import { useAuth } from "../hooks/useAuth";

const API_URL = "http://localhost:3001";

const LEVEL_LABELS: Record<string, string> = {
  beginner: "Beginner",
  intermediate: "Intermediate",
  advanced: "Advanced",
};

type AssessmentData = {
  devExperience: string;
  pythonProficiency: string;
  roboticsBackground: string;
  rosExposure: string;
  learningGoals: string[];
  computedLevel: string;
};

export default function ProfilePage() {
  const { user, refreshSession, setAssessmentLevel } = useAuth();
  const [displayName, setDisplayName] = useState("");
  const [isSaving, setIsSaving] = useState(false);
  const [saved, setSaved] = useState(false);
  const [showDeleteConfirm, setShowDeleteConfirm] = useState(false);
  const [deletePassword, setDeletePassword] = useState("");
  const [deleteError, setDeleteError] = useState("");
  const [assessment, setAssessment] = useState<AssessmentData | null>(null);
  const [showRetakeForm, setShowRetakeForm] = useState(false);
  const [isAssessmentSubmitting, setIsAssessmentSubmitting] = useState(false);

  useEffect(() => {
    if (user) {
      setDisplayName(user.displayName || "");
    }
  }, [user]);

  useEffect(() => {
    (async () => {
      try {
        const res = await fetch(`${API_URL}/api/assessment`, {
          credentials: "include",
        });
        if (res.ok) {
          setAssessment(await res.json());
        }
      } catch {
        // No assessment yet
      }
    })();
  }, []);

  const handleAssessmentSubmit = async (data: Record<string, unknown>) => {
    setIsAssessmentSubmitting(true);
    try {
      const res = await fetch(`${API_URL}/api/assessment`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify(data),
      });
      if (res.ok) {
        const result = await res.json();
        setAssessment(result.assessment);
        setAssessmentLevel(result.assessment.computedLevel);
        setShowRetakeForm(false);
      }
    } catch {
      // Error handled by form
    } finally {
      setIsAssessmentSubmitting(false);
    }
  };

  const handleSave = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSaving(true);
    setSaved(false);
    try {
      await fetch(`${API_URL}/api/profile`, {
        method: "PATCH",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ displayName }),
      });
      await refreshSession();
      setSaved(true);
    } catch {
      // silently fail
    } finally {
      setIsSaving(false);
    }
  };

  const handleDelete = async () => {
    setDeleteError("");
    try {
      const res = await fetch(`${API_URL}/api/auth/delete-user`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify({ password: deletePassword }),
      });
      if (res.ok) {
        window.location.href = "/";
      } else {
        setDeleteError("Incorrect password or deletion failed.");
      }
    } catch {
      setDeleteError("An error occurred.");
    }
  };

  return (
    <ProtectedRoute title="Profile" description="Manage your profile">
      <div style={{ maxWidth: 600, margin: "2rem auto", padding: "0 1rem" }}>
        <h1>Your Profile</h1>

        {user && (
          <div
            style={{
              padding: "1rem",
              background: "var(--ifm-background-surface-color)",
              border: "1px solid var(--ifm-color-emphasis-200)",
              borderRadius: 8,
              marginBottom: "2rem",
            }}
          >
            <p><strong>Email:</strong> {user.email}</p>
            <p><strong>Member since:</strong> {new Date().toLocaleDateString()}</p>
          </div>
        )}

        <h2>Edit Display Name</h2>
        <form onSubmit={handleSave} style={{ marginBottom: "2rem" }}>
          <div style={{ marginBottom: "1rem" }}>
            <label htmlFor="displayName" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Display Name
            </label>
            <input
              id="displayName"
              type="text"
              value={displayName}
              onChange={(e) => { setDisplayName(e.target.value); setSaved(false); }}
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: "1px solid #d1d5db" }}
            />
          </div>
          <button
            type="submit"
            disabled={isSaving}
            style={{
              padding: "0.5rem 1.5rem",
              background: "#2e8555",
              color: "#fff",
              border: "none",
              borderRadius: 6,
              cursor: "pointer",
            }}
          >
            {isSaving ? "Saving..." : "Save Changes"}
          </button>
          {saved && <span style={{ marginLeft: "1rem", color: "#16a34a" }}>Saved!</span>}
        </form>

        <h2>Background Assessment</h2>
        {assessment ? (
          <div
            style={{
              padding: "1rem",
              background: "var(--ifm-background-surface-color)",
              border: "1px solid var(--ifm-color-emphasis-200)",
              borderRadius: 8,
              marginBottom: "1rem",
            }}
          >
            <p>
              <strong>Computed Level:</strong>{" "}
              <span
                style={{
                  padding: "0.125rem 0.5rem",
                  background: "#dbeafe",
                  color: "#1d4ed8",
                  borderRadius: 4,
                  fontSize: "0.875rem",
                  fontWeight: 600,
                }}
              >
                {LEVEL_LABELS[assessment.computedLevel] || assessment.computedLevel}
              </span>
            </p>
            <p><strong>Dev Experience:</strong> {assessment.devExperience}</p>
            <p><strong>Python Proficiency:</strong> {assessment.pythonProficiency}</p>
            <p><strong>Robotics Background:</strong> {assessment.roboticsBackground}</p>
            <p><strong>ROS Exposure:</strong> {assessment.rosExposure}</p>
            <p><strong>Learning Goals:</strong> {(assessment.learningGoals || []).join(", ")}</p>

            {showRetakeForm ? (
              <div style={{ marginTop: "1rem" }}>
                <AssessmentForm
                  initialData={assessment}
                  onSubmit={handleAssessmentSubmit}
                  isSubmitting={isAssessmentSubmitting}
                />
                <button
                  onClick={() => setShowRetakeForm(false)}
                  style={{
                    marginTop: "0.5rem",
                    padding: "0.5rem 1rem",
                    background: "none",
                    border: "1px solid #d1d5db",
                    borderRadius: 6,
                    cursor: "pointer",
                  }}
                >
                  Cancel
                </button>
              </div>
            ) : (
              <button
                onClick={() => setShowRetakeForm(true)}
                style={{
                  marginTop: "0.5rem",
                  padding: "0.5rem 1.5rem",
                  background: "#2e8555",
                  color: "#fff",
                  border: "none",
                  borderRadius: 6,
                  cursor: "pointer",
                }}
              >
                Retake Assessment
              </button>
            )}
          </div>
        ) : (
          <p>
            <a href="/assessment">Complete your background assessment</a> to get
            personalized content.
          </p>
        )}

        <h2 style={{ color: "#dc2626", marginTop: "3rem" }}>Danger Zone</h2>
        <div
          style={{
            padding: "1rem",
            border: "1px solid #fca5a5",
            borderRadius: 8,
          }}
        >
          <p>
            Deleting your account will remove all your data within 30 days. This action cannot be undone.
          </p>
          {!showDeleteConfirm ? (
            <button
              onClick={() => setShowDeleteConfirm(true)}
              style={{
                padding: "0.5rem 1.5rem",
                background: "#dc2626",
                color: "#fff",
                border: "none",
                borderRadius: 6,
                cursor: "pointer",
              }}
            >
              Delete Account
            </button>
          ) : (
            <div>
              <p style={{ fontWeight: 600 }}>Enter your password to confirm:</p>
              <input
                type="password"
                value={deletePassword}
                onChange={(e) => setDeletePassword(e.target.value)}
                placeholder="Your password"
                style={{ padding: "0.5rem", borderRadius: 4, border: "1px solid #d1d5db", marginRight: "0.5rem" }}
              />
              <button
                onClick={handleDelete}
                style={{
                  padding: "0.5rem 1rem",
                  background: "#dc2626",
                  color: "#fff",
                  border: "none",
                  borderRadius: 6,
                  cursor: "pointer",
                  marginRight: "0.5rem",
                }}
              >
                Confirm Delete
              </button>
              <button
                onClick={() => { setShowDeleteConfirm(false); setDeletePassword(""); setDeleteError(""); }}
                style={{
                  padding: "0.5rem 1rem",
                  background: "none",
                  border: "1px solid #d1d5db",
                  borderRadius: 6,
                  cursor: "pointer",
                }}
              >
                Cancel
              </button>
              {deleteError && <p style={{ color: "#dc2626", marginTop: "0.5rem" }}>{deleteError}</p>}
            </div>
          )}
        </div>
      </div>
    </ProtectedRoute>
  );
}
