import React, { useState } from "react";
import { useAuthContext } from "../../../components/AuthProvider";
import NavbarContentOrig from "@theme-original/Navbar/Content";

export default function NavbarContentWrapper() {
  const { user, isAuthenticated, isLoading } = useAuthContext();
  const [menuOpen, setMenuOpen] = useState(false);

  return (
    <>
      <NavbarContentOrig />
      <div
        style={{
          display: "flex",
          alignItems: "center",
          gap: "0.5rem",
          marginLeft: "auto",
          paddingRight: "1rem",
        }}
      >
        {isLoading ? null : isAuthenticated && user ? (
          <div style={{ position: "relative" }}>
            <button
              onClick={() => setMenuOpen(!menuOpen)}
              style={{
                background: "#2e8555",
                color: "#fff",
                border: "none",
                borderRadius: "50%",
                width: 36,
                height: 36,
                cursor: "pointer",
                fontSize: "0.875rem",
                fontWeight: 600,
              }}
              aria-label="User menu"
              aria-expanded={menuOpen}
            >
              {user.displayName?.[0]?.toUpperCase() || "U"}
            </button>
            {menuOpen && (
              <div
                style={{
                  position: "absolute",
                  right: 0,
                  top: "100%",
                  marginTop: 8,
                  background: "var(--ifm-background-color)",
                  border: "1px solid var(--ifm-color-emphasis-300)",
                  borderRadius: 8,
                  boxShadow: "0 4px 12px rgba(0,0,0,0.1)",
                  minWidth: 180,
                  zIndex: 100,
                  padding: "0.5rem 0",
                }}
              >
                <div
                  style={{
                    padding: "0.5rem 1rem",
                    borderBottom: "1px solid var(--ifm-color-emphasis-200)",
                    fontSize: "0.875rem",
                    fontWeight: 600,
                  }}
                >
                  {user.displayName}
                </div>
                <a
                  href="/dashboard"
                  style={{ display: "block", padding: "0.5rem 1rem", textDecoration: "none" }}
                >
                  Dashboard
                </a>
                <a
                  href="/profile"
                  style={{ display: "block", padding: "0.5rem 1rem", textDecoration: "none" }}
                >
                  Profile
                </a>
                <button
                  onClick={async () => {
                    await fetch("http://localhost:3001/api/auth/sign-out", {
                      method: "POST",
                      credentials: "include",
                    });
                    window.location.href = "/";
                  }}
                  style={{
                    display: "block",
                    width: "100%",
                    textAlign: "left",
                    padding: "0.5rem 1rem",
                    background: "none",
                    border: "none",
                    cursor: "pointer",
                    color: "#dc2626",
                    borderTop: "1px solid var(--ifm-color-emphasis-200)",
                  }}
                >
                  Sign Out
                </button>
              </div>
            )}
          </div>
        ) : (
          <>
            <a
              href="/auth/signin"
              style={{
                padding: "0.375rem 0.75rem",
                borderRadius: 6,
                textDecoration: "none",
                fontSize: "0.875rem",
              }}
            >
              Sign In
            </a>
            <a
              href="/auth/signup"
              style={{
                padding: "0.375rem 0.75rem",
                background: "#2e8555",
                color: "#fff",
                borderRadius: 6,
                textDecoration: "none",
                fontSize: "0.875rem",
              }}
            >
              Sign Up
            </a>
          </>
        )}
      </div>
    </>
  );
}
