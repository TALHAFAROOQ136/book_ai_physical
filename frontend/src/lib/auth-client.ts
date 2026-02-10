import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL:
    typeof window !== "undefined"
      ? (window as any).__AUTH_API_URL__ || "http://localhost:3001"
      : "http://localhost:3001",
});
