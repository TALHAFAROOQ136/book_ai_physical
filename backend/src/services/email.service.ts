import { Resend } from "resend";

const resend = new Resend(process.env.RESEND_API_KEY);

const FROM_EMAIL = "noreply@textbook.example.com";

export async function sendPasswordResetEmail(
  to: string,
  resetUrl: string
): Promise<void> {
  // Fire-and-forget to prevent timing attacks (per Better-Auth docs)
  void resend.emails.send({
    from: FROM_EMAIL,
    to,
    subject: "Reset your password — Physical AI Textbook",
    html: `
      <h2>Password Reset Request</h2>
      <p>You requested a password reset for your Physical AI Textbook account.</p>
      <p><a href="${resetUrl}" style="display:inline-block;padding:12px 24px;background:#2e8555;color:#fff;text-decoration:none;border-radius:6px;">Reset Password</a></p>
      <p>If you didn't request this, you can safely ignore this email.</p>
      <p>This link expires in 1 hour.</p>
    `,
  });
}
