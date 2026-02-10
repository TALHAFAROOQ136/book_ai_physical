import React, { useState } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../../hooks/useAuth";

export default function ResetPasswordPage() {
  const { resetPassword } = useAuth();
  const [password, setPassword] = useState("");
  const [confirm, setConfirm] = useState("");
  const [error, setError] = useState("");
  const [success, setSuccess] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");

    if (password.length < 8) {
      setError("Password must be at least 8 characters.");
      return;
    }
    if (!/[a-zA-Z]/.test(password)) {
      setError("Password must contain at least one letter.");
      return;
    }
    if (!/[0-9]/.test(password)) {
      setError("Password must contain at least one number.");
      return;
    }
    if (password !== confirm) {
      setError("Passwords do not match.");
      return;
    }

    const params = new URLSearchParams(window.location.search);
    const token = params.get("token");
    if (!token) {
      setError("Invalid or missing reset token. Please request a new reset link.");
      return;
    }

    setIsSubmitting(true);
    try {
      const result = await resetPassword(password, token);
      if (result.error) {
        setError("Reset failed. The link may have expired. Please request a new one.");
      } else {
        setSuccess(true);
      }
    } catch {
      setError("An unexpected error occurred.");
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout title="Reset Password" description="Set a new password">
      <div style={{ maxWidth: 420, margin: "3rem auto", padding: "0 1rem" }}>
        <h1>Reset Password</h1>

        {success ? (
          <div
            style={{
              padding: "1rem",
              background: "#f0fdf4",
              border: "1px solid #86efac",
              borderRadius: 6,
            }}
          >
            <p style={{ margin: 0 }}>
              Your password has been reset successfully.
            </p>
            <p style={{ marginTop: "1rem" }}>
              <a href="/auth/signin">Sign in with your new password</a>
            </p>
          </div>
        ) : (
          <>
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

            <form onSubmit={handleSubmit} noValidate>
              <div style={{ marginBottom: "1rem" }}>
                <label htmlFor="password" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
                  New Password
                </label>
                <input
                  id="password"
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                  aria-required="true"
                  aria-describedby="password-hint"
                  autoComplete="new-password"
                  style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: "1px solid #d1d5db" }}
                />
                <small id="password-hint" style={{ color: "#6b7280" }}>
                  Minimum 8 characters, at least 1 letter and 1 number.
                </small>
              </div>

              <div style={{ marginBottom: "1rem" }}>
                <label htmlFor="confirm" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
                  Confirm New Password
                </label>
                <input
                  id="confirm"
                  type="password"
                  value={confirm}
                  onChange={(e) => setConfirm(e.target.value)}
                  required
                  aria-required="true"
                  autoComplete="new-password"
                  style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: "1px solid #d1d5db" }}
                />
              </div>

              <button
                type="submit"
                disabled={isSubmitting}
                style={{
                  width: "100%",
                  padding: "0.75rem",
                  background: "#2e8555",
                  color: "#fff",
                  border: "none",
                  borderRadius: 6,
                  fontSize: "1rem",
                  cursor: isSubmitting ? "not-allowed" : "pointer",
                  opacity: isSubmitting ? 0.7 : 1,
                }}
              >
                {isSubmitting ? "Resetting..." : "Reset Password"}
              </button>
            </form>
          </>
        )}
      </div>
    </Layout>
  );
}
