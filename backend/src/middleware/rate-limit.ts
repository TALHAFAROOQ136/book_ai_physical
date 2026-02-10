import type { MiddlewareHandler } from "hono";

type RateLimitOptions = {
  windowMs: number;
  max: number;
};

const store = new Map<string, { count: number; resetAt: number }>();

// Cleanup expired entries every 5 minutes
setInterval(() => {
  const now = Date.now();
  for (const [key, entry] of store) {
    if (entry.resetAt <= now) store.delete(key);
  }
}, 5 * 60_000);

export function rateLimiter(opts: RateLimitOptions): MiddlewareHandler {
  return async (c, next) => {
    const ip =
      c.req.header("x-forwarded-for")?.split(",")[0]?.trim() ||
      c.req.header("x-real-ip") ||
      "unknown";

    const path = new URL(c.req.url).pathname;
    const key = `${ip}:${path.startsWith("/api/auth") ? "auth" : "api"}`;

    const now = Date.now();
    let entry = store.get(key);

    if (!entry || entry.resetAt <= now) {
      entry = { count: 0, resetAt: now + opts.windowMs };
      store.set(key, entry);
    }

    entry.count++;

    c.header("X-RateLimit-Limit", String(opts.max));
    c.header("X-RateLimit-Remaining", String(Math.max(0, opts.max - entry.count)));
    c.header("X-RateLimit-Reset", String(Math.ceil(entry.resetAt / 1000)));

    if (entry.count > opts.max) {
      return c.json(
        { message: "Too many requests. Please try again later.", code: "RATE_LIMITED" },
        429
      );
    }

    await next();
  };
}
