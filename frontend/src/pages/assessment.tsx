import React, { useState, useEffect } from "react";
import ProtectedRoute from "../components/ProtectedRoute";
import AssessmentForm from "../components/AssessmentForm";
import { useAuth } from "../hooks/useAuth";

const API_URL = "http://localhost:3001";

export default function AssessmentPage() {
  const { setAssessmentLevel } = useAuth();
  const [existingData, setExistingData] = useState(null);
  const [recommendation, setRecommendation] = useState<{
    startingChapter: string;
    reason: string;
  } | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    (async () => {
      try {
        const res = await fetch(`${API_URL}/api/assessment`, {
          credentials: "include",
        });
        if (res.ok) {
          const data = await res.json();
          setExistingData(data);
        }
      } catch {
        // No existing assessment
      } finally {
        setIsLoading(false);
      }
    })();
  }, []);

  const handleSubmit = async (data: Record<string, unknown>) => {
    setIsSubmitting(true);
    try {
      const res = await fetch(`${API_URL}/api/assessment`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        credentials: "include",
        body: JSON.stringify(data),
      });
      if (res.ok) {
        const result = await res.json();
        setAssessmentLevel(result.assessment.computedLevel);
        setRecommendation(result.recommendation);
      }
    } catch {
      // Error handled by form
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <ProtectedRoute title="Background Assessment" description="Tell us about your background">
      <div style={{ maxWidth: 600, margin: "2rem auto", padding: "0 1rem" }}>
        <h1>Background Assessment</h1>
        <p>
          Help us personalize your learning experience. This takes about 2
          minutes. You can update your answers anytime from your profile.
        </p>

        {recommendation ? (
          <div
            style={{
              padding: "1.5rem",
              background: "#f0fdf4",
              border: "1px solid #86efac",
              borderRadius: 8,
              marginBottom: "2rem",
            }}
          >
            <h3 style={{ marginTop: 0 }}>Your personalized recommendation</h3>
            <p>{recommendation.reason}</p>
            <a
              href={`/chapters/${recommendation.startingChapter}`}
              style={{
                display: "inline-block",
                padding: "0.5rem 1.5rem",
                background: "#2e8555",
                color: "#fff",
                borderRadius: 6,
                textDecoration: "none",
                marginRight: "1rem",
              }}
            >
              Start Learning
            </a>
            <a href="/dashboard">Go to Dashboard</a>
          </div>
        ) : null}

        {isLoading ? (
          <p>Loading...</p>
        ) : (
          <>
            <AssessmentForm
              initialData={existingData || undefined}
              onSubmit={handleSubmit}
              isSubmitting={isSubmitting}
            />
            {!existingData && (
              <p style={{ marginTop: "1rem", textAlign: "center" }}>
                <a href="/dashboard">Skip for now</a> — you can always take
                this later from your profile.
              </p>
            )}
          </>
        )}
      </div>
    </ProtectedRoute>
  );
}
