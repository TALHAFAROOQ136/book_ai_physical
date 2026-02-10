import React, { createContext, useContext, useEffect, useState, useRef, useCallback } from "react";
import { authClient } from "../lib/auth-client";

type User = {
  id: string;
  email: string;
  name: string;
  displayName: string;
};

type Session = {
  id: string;
  userId: string;
  expiresAt: string;
};

type AssessmentLevel = "beginner" | "intermediate" | "advanced";

type ChapterStatus = "not_started" | "in_progress" | "completed";

type AuthContextType = {
  user: User | null;
  session: Session | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  assessmentLevel: AssessmentLevel;
  setAssessmentLevel: (level: AssessmentLevel) => void;
  refreshSession: () => Promise<void>;
  chapterStatuses: Record<string, ChapterStatus>;
};

const AuthContext = createContext<AuthContextType>({
  user: null,
  session: null,
  isAuthenticated: false,
  isLoading: true,
  assessmentLevel: "intermediate",
  setAssessmentLevel: () => {},
  refreshSession: async () => {},
  chapterStatuses: {},
});

const API_URL = "http://localhost:3001";

function redirectToSignIn() {
  if (typeof window === "undefined") return;
  const returnUrl = window.location.pathname + window.location.search;
  window.location.href = `/auth/signin?returnUrl=${encodeURIComponent(returnUrl)}`;
}

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [session, setSession] = useState<Session | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [assessmentLevel, setAssessmentLevel] =
    useState<AssessmentLevel>("intermediate");
  const [chapterStatuses, setChapterStatuses] = useState<Record<string, ChapterStatus>>({});
  const wasAuthenticated = useRef(false);

  const fetchSession = useCallback(async () => {
    try {
      const response = await fetch(`${API_URL}/api/auth/get-session`, {
        credentials: "include",
      });
      if (response.ok) {
        const data = await response.json();
        if (data.session && data.user) {
          setUser(data.user as User);
          setSession(data.session as Session);
          wasAuthenticated.current = true;

          // Fetch assessment level and progress in parallel
          const [assessmentRes, progressRes] = await Promise.all([
            fetch(`${API_URL}/api/assessment`, { credentials: "include" }),
            fetch(`${API_URL}/api/progress`, { credentials: "include" }),
          ]);
          if (assessmentRes.ok) {
            const assessment = await assessmentRes.json();
            setAssessmentLevel(
              (assessment.computedLevel as AssessmentLevel) || "intermediate"
            );
          }
          if (progressRes.ok) {
            const progressData = await progressRes.json();
            const statuses: Record<string, ChapterStatus> = {};
            for (const ch of progressData.chapters || []) {
              statuses[ch.chapterId] = ch.status as ChapterStatus;
            }
            setChapterStatuses(statuses);
          }
        } else {
          // Session expired — redirect if user was previously authenticated
          if (wasAuthenticated.current) {
            wasAuthenticated.current = false;
            setUser(null);
            setSession(null);
            redirectToSignIn();
            return;
          }
          setUser(null);
          setSession(null);
        }
      } else if (response.status === 401 && wasAuthenticated.current) {
        // Explicit 401 — session expired
        wasAuthenticated.current = false;
        setUser(null);
        setSession(null);
        redirectToSignIn();
        return;
      } else {
        setUser(null);
        setSession(null);
      }
    } catch {
      setUser(null);
      setSession(null);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchSession();

    // Re-check session when tab regains focus (detects expiry across tabs)
    const handleVisibilityChange = () => {
      if (document.visibilityState === "visible" && wasAuthenticated.current) {
        fetchSession();
      }
    };
    document.addEventListener("visibilitychange", handleVisibilityChange);
    return () => document.removeEventListener("visibilitychange", handleVisibilityChange);
  }, [fetchSession]);

  return (
    <AuthContext.Provider
      value={{
        user,
        session,
        isAuthenticated: !!user,
        isLoading,
        assessmentLevel,
        setAssessmentLevel,
        refreshSession: fetchSession,
        chapterStatuses,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuthContext() {
  return useContext(AuthContext);
}

export { AuthContext };
