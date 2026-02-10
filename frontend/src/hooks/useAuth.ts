import { useAuthContext } from "../components/AuthProvider";
import { authClient } from "../lib/auth-client";

export function useAuth() {
  const context = useAuthContext();

  const signUp = async (data: {
    name: string;
    email: string;
    password: string;
    displayName: string;
  }) => {
    const result = await authClient.signUp.email({
      name: data.name,
      email: data.email,
      password: data.password,
      displayName: data.displayName,
    });
    if (!result.error) {
      await context.refreshSession();
    }
    return result;
  };

  const signIn = async (data: {
    email: string;
    password: string;
    rememberMe?: boolean;
  }) => {
    const result = await authClient.signIn.email({
      email: data.email,
      password: data.password,
      rememberMe: data.rememberMe,
    });
    if (!result.error) {
      await context.refreshSession();
    }
    return result;
  };

  const signOut = async () => {
    await authClient.signOut();
    await context.refreshSession();
  };

  const forgetPassword = async (email: string, redirectTo: string) => {
    return authClient.forgetPassword({ email, redirectTo });
  };

  const resetPassword = async (newPassword: string, token: string) => {
    return authClient.resetPassword({ newPassword, token });
  };

  return {
    ...context,
    signUp,
    signIn,
    signOut,
    forgetPassword,
    resetPassword,
  };
}
