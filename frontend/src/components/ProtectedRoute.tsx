import React, { useEffect } from "react";
import { useAuthContext } from "./AuthProvider";
import Layout from "@theme/Layout";

type ProtectedRouteProps = {
  children: React.ReactNode;
  title?: string;
  description?: string;
};

export default function ProtectedRoute({
  children,
  title,
  description,
}: ProtectedRouteProps) {
  const { isAuthenticated, isLoading } = useAuthContext();

  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      const returnUrl = encodeURIComponent(window.location.pathname);
      window.location.href = `/auth/signin?returnUrl=${returnUrl}`;
    }
  }, [isAuthenticated, isLoading]);

  if (isLoading) {
    return (
      <Layout title={title} description={description}>
        <div
          style={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            minHeight: "50vh",
          }}
        >
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return null;
  }

  return (
    <Layout title={title} description={description}>
      {children}
    </Layout>
  );
}
