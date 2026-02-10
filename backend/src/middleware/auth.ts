import type { Context, Next } from "hono";
import { auth } from "../auth/index.js";

export type AuthUser = {
  id: string;
  email: string;
  name: string;
  displayName: string;
};

/**
 * Middleware that validates the session cookie and attaches the authenticated
 * user to the Hono context. Returns 401 if no valid session exists.
 */
export async function requireAuth(c: Context, next: Next) {
  const session = await auth.api.getSession({
    headers: c.req.raw.headers,
  });

  if (!session) {
    return c.json({ message: "Unauthorized", code: "UNAUTHORIZED" }, 401);
  }

  c.set("user", session.user as AuthUser);
  c.set("session", session.session);
  await next();
}
