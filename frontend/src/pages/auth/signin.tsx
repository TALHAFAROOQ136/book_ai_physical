import React, { useState } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../../hooks/useAuth";

export default function SignInPage() {
  const { signIn, isAuthenticated } = useAuth();
  const [form, setForm] = useState({ email: "", password: "" });
  const [rememberMe, setRememberMe] = useState(false);
  const [error, setError] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  if (isAuthenticated) {
    if (typeof window !== "undefined") {
      const params = new URLSearchParams(window.location.search);
      window.location.href = params.get("returnUrl") || "/dashboard";
    }
    return null;
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError("");
    setIsSubmitting(true);

    try {
      const result = await signIn({
        email: form.email,
        password: form.password,
        rememberMe,
      });
      if (result.error) {
        setError("Invalid email or password. Please try again.");
      } else {
        const params = new URLSearchParams(window.location.search);
        window.location.href = params.get("returnUrl") || "/dashboard";
      }
    } catch {
      setError("An unexpected error occurred. Please try again.");
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div style={{ maxWidth: 420, margin: "3rem auto", padding: "0 1rem" }}>
        <h1>Sign In</h1>
        <p>Welcome back! Sign in to continue your learning journey.</p>

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
            <label htmlFor="email" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Email
            </label>
            <input
              id="email"
              type="email"
              value={form.email}
              onChange={(e) => setForm((f) => ({ ...f, email: e.target.value }))}
              required
              aria-required="true"
              autoComplete="email"
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: "1px solid #d1d5db" }}
            />
          </div>

          <div style={{ marginBottom: "1rem" }}>
            <label htmlFor="password" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Password
            </label>
            <input
              id="password"
              type="password"
              value={form.password}
              onChange={(e) =>
                setForm((f) => ({ ...f, password: e.target.value }))
              }
              required
              aria-required="true"
              autoComplete="current-password"
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: "1px solid #d1d5db" }}
            />
          </div>

          <div
            style={{
              display: "flex",
              justifyContent: "space-between",
              alignItems: "center",
              marginBottom: "1rem",
            }}
          >
            <label style={{ display: "flex", alignItems: "center", gap: 6 }}>
              <input
                type="checkbox"
                checked={rememberMe}
                onChange={(e) => setRememberMe(e.target.checked)}
              />
              Remember me
            </label>
            <a href="/auth/forgot-password">Forgot password?</a>
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
            {isSubmitting ? "Signing in..." : "Sign In"}
          </button>
        </form>

        <p style={{ marginTop: "1.5rem", textAlign: "center" }}>
          Don't have an account? <a href="/auth/signup">Sign up</a>
        </p>
      </div>
    </Layout>
  );
}
