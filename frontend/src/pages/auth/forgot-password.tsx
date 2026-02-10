import React, { useState } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../../hooks/useAuth";

export default function ForgotPasswordPage() {
  const { forgetPassword } = useAuth();
  const [email, setEmail] = useState("");
  const [sent, setSent] = useState(false);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSubmitting(true);
    try {
      await forgetPassword(email, `${window.location.origin}/auth/reset-password`);
    } catch {
      // Always show success to prevent email enumeration
    }
    setSent(true);
    setIsSubmitting(false);
  };

  return (
    <Layout title="Forgot Password" description="Reset your password">
      <div style={{ maxWidth: 420, margin: "3rem auto", padding: "0 1rem" }}>
        <h1>Forgot Password</h1>

        {sent ? (
          <div
            role="status"
            aria-live="polite"
            style={{
              padding: "1rem",
              background: "#f0fdf4",
              border: "1px solid #86efac",
              borderRadius: 6,
            }}
          >
            <p style={{ margin: 0 }}>
              If an account with that email exists, we've sent a password reset
              link. Please check your inbox.
            </p>
            <p style={{ marginTop: "1rem" }}>
              <a href="/auth/signin">Return to sign in</a>
            </p>
          </div>
        ) : (
          <>
            <p>Enter your email address and we'll send you a reset link.</p>
            <form onSubmit={handleSubmit} noValidate>
              <div style={{ marginBottom: "1rem" }}>
                <label htmlFor="email" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
                  Email
                </label>
                <input
                  id="email"
                  type="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                  autoComplete="email"
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
                {isSubmitting ? "Sending..." : "Send Reset Link"}
              </button>
            </form>
            <p style={{ marginTop: "1.5rem", textAlign: "center" }}>
              <a href="/auth/signin">Back to sign in</a>
            </p>
          </>
        )}
      </div>
    </Layout>
  );
}
