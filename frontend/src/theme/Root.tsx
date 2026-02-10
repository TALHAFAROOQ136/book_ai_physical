import React from "react";
import { AuthProvider } from "../components/AuthProvider";

export default function Root({ children }: { children: React.ReactNode }) {
  return <AuthProvider>{children}</AuthProvider>;
}
