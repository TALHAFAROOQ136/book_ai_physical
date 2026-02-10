import "dotenv/config";
import { Hono } from "hono";
import { cors } from "hono/cors";
import { serve } from "@hono/node-server";
import { auth } from "./auth/index.js";
import { assessmentRoutes } from "./routes/assessment.js";
import { progressRoutes } from "./routes/progress.js";
import { learningPathRoutes } from "./routes/learning-path.js";
import { profileRoutes } from "./routes/profile.js";
import { rateLimiter } from "./middleware/rate-limit.js";

const app = new Hono();

// CORS configuration — uses production domain when set, falls back to localhost
const allowedOrigins = process.env.FRONTEND_URL
  ? process.env.FRONTEND_URL.split(",").map((o) => o.trim())
  : ["http://localhost:3000"];

app.use(
  "/api/*",
  cors({
    origin: allowedOrigins,
    credentials: true,
    allowHeaders: ["Content-Type", "Authorization"],
    allowMethods: ["GET", "POST", "PATCH", "DELETE", "OPTIONS"],
  })
);

// Rate limiting — stricter on auth endpoints
app.use("/api/auth/*", rateLimiter({ windowMs: 60_000, max: 10 }));
app.use("/api/*", rateLimiter({ windowMs: 60_000, max: 60 }));

// Mount Better-Auth handler for all /api/auth/* routes
app.on(["POST", "GET"], "/api/auth/**", (c) => {
  return auth.handler(c.req.raw);
});

// Mount custom API routes
app.route("/api/assessment", assessmentRoutes);
app.route("/api/progress", progressRoutes);
app.route("/api/learning-path", learningPathRoutes);
app.route("/api/profile", profileRoutes);

// Health check
app.get("/api/health", (c) => {
  return c.json({ status: "ok", timestamp: new Date().toISOString() });
});

const port = parseInt(process.env.PORT || "3001", 10);

console.log(`Auth API server running on http://localhost:${port}`);

serve({
  fetch: app.fetch,
  port,
});

export default app;
