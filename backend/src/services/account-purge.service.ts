import { lt, isNotNull, and } from "drizzle-orm";
import { db } from "../db/index.js";
import { user } from "../db/schema.js";

const PURGE_DAYS = 30;

/**
 * Purge users that were soft-deleted more than 30 days ago.
 * Cascade rules on the schema handle child table cleanup.
 */
export async function purgeDeletedAccounts(): Promise<number> {
  const cutoff = new Date();
  cutoff.setDate(cutoff.getDate() - PURGE_DAYS);

  const deleted = await db
    .delete(user)
    .where(and(isNotNull(user.deletedAt), lt(user.deletedAt, cutoff)))
    .returning({ id: user.id });

  return deleted.length;
}
