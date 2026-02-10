import React, { useState } from "react";
import Layout from "@theme/Layout";
import { useAuth } from "../../hooks/useAuth";

function validatePassword(password: string): string | null {
  if (password.length < 8) return "Password must be at least 8 characters.";
  if (!/[a-zA-Z]/.test(password))
    return "Password must contain at least one letter.";
  if (!/[0-9]/.test(password))
    return "Password must contain at least one number.";
  return null;
}

function validateEmail(email: string): string | null {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  if (!emailRegex.test(email)) return "Please enter a valid email address.";
  return null;
}

export default function SignUpPage() {
  const { signUp, isAuthenticated } = useAuth();
  const [form, setForm] = useState({
    name: "",
    email: "",
    password: "",
    displayName: "",
  });
  const [errors, setErrors] = useState<Record<string, string>>({});
  const [serverError, setServerError] = useState("");
  const [isSubmitting, setIsSubmitting] = useState(false);

  if (isAuthenticated) {
    if (typeof window !== "undefined") {
      window.location.href = "/assessment";
    }
    return null;
  }

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setForm((prev) => ({ ...prev, [name]: value }));
    setErrors((prev) => ({ ...prev, [name]: "" }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setServerError("");

    const newErrors: Record<string, string> = {};
    if (!form.displayName.trim())
      newErrors.displayName = "Display name is required.";
    if (!form.name.trim()) newErrors.name = "Full name is required.";
    const emailErr = validateEmail(form.email);
    if (emailErr) newErrors.email = emailErr;
    const pwErr = validatePassword(form.password);
    if (pwErr) newErrors.password = pwErr;

    if (Object.keys(newErrors).length > 0) {
      setErrors(newErrors);
      return;
    }

    setIsSubmitting(true);
    try {
      const result = await signUp(form);
      if (result.error) {
        setServerError(
          result.error.message || "Registration failed. Please try again."
        );
      } else {
        window.location.href = "/assessment";
      }
    } catch {
      setServerError("An unexpected error occurred. Please try again.");
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create your account">
      <div
        style={{
          maxWidth: 420,
          margin: "3rem auto",
          padding: "0 1rem",
        }}
      >
        <h1>Create Account</h1>
        <p>
          Join the Physical AI Textbook to track your progress and get
          personalized content.
        </p>

        {serverError && (
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
            {serverError}
          </div>
        )}

        <form onSubmit={handleSubmit} noValidate>
          <div style={{ marginBottom: "1rem" }}>
            <label htmlFor="displayName" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Display Name
            </label>
            <input
              id="displayName"
              name="displayName"
              type="text"
              value={form.displayName}
              onChange={handleChange}
              required
              autoComplete="nickname"
              aria-invalid={!!errors.displayName}
              aria-describedby={errors.displayName ? "displayName-error" : undefined}
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: `1px solid ${errors.displayName ? "#dc2626" : "#d1d5db"}` }}
            />
            {errors.displayName && (
              <small id="displayName-error" role="alert" style={{ color: "#dc2626" }}>{errors.displayName}</small>
            )}
          </div>

          <div style={{ marginBottom: "1rem" }}>
            <label htmlFor="name" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Full Name
            </label>
            <input
              id="name"
              name="name"
              type="text"
              value={form.name}
              onChange={handleChange}
              required
              autoComplete="name"
              aria-invalid={!!errors.name}
              aria-describedby={errors.name ? "name-error" : undefined}
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: `1px solid ${errors.name ? "#dc2626" : "#d1d5db"}` }}
            />
            {errors.name && (
              <small id="name-error" role="alert" style={{ color: "#dc2626" }}>{errors.name}</small>
            )}
          </div>

          <div style={{ marginBottom: "1rem" }}>
            <label htmlFor="email" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Email
            </label>
            <input
              id="email"
              name="email"
              type="email"
              value={form.email}
              onChange={handleChange}
              required
              autoComplete="email"
              aria-invalid={!!errors.email}
              aria-describedby={errors.email ? "email-error" : undefined}
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: `1px solid ${errors.email ? "#dc2626" : "#d1d5db"}` }}
            />
            {errors.email && (
              <small id="email-error" role="alert" style={{ color: "#dc2626" }}>{errors.email}</small>
            )}
          </div>

          <div style={{ marginBottom: "1rem" }}>
            <label htmlFor="password" style={{ display: "block", marginBottom: 4, fontWeight: 600 }}>
              Password
            </label>
            <input
              id="password"
              name="password"
              type="password"
              value={form.password}
              onChange={handleChange}
              required
              autoComplete="new-password"
              aria-invalid={!!errors.password}
              aria-describedby={`password-hint${errors.password ? " password-error" : ""}`}
              style={{ width: "100%", padding: "0.5rem", borderRadius: 4, border: `1px solid ${errors.password ? "#dc2626" : "#d1d5db"}` }}
            />
            {errors.password && (
              <small id="password-error" role="alert" style={{ color: "#dc2626" }}>{errors.password}</small>
            )}
            <small id="password-hint" style={{ color: "#6b7280" }}>
              Minimum 8 characters, at least 1 letter and 1 number.
            </small>
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
            {isSubmitting ? "Creating account..." : "Sign Up"}
          </button>
        </form>

        <p style={{ marginTop: "1.5rem", textAlign: "center" }}>
          Already have an account?{" "}
          <a href="/auth/signin">Sign in</a>
        </p>
      </div>
    </Layout>
  );
}
