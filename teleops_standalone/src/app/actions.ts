'use server'

import { db } from "@/db";
import { robots, teleopSessions, operatorProfiles } from "@/db/schema";
import { eq } from "drizzle-orm";
import { revalidatePath } from "next/cache";

export async function getRobots() {
  return await db.select().from(robots);
}

export async function getRobot(id: string) {
  const result = await db.select().from(robots).where(eq(robots.id, id));
  return result[0];
}

export async function createRobot(formData: FormData) {
  const robotName = formData.get("robotName") as string;
  const robotType = formData.get("robotType") as string;
  const backendType = formData.get("backendType") as string;
  const host = formData.get("host") as string;
  const port = formData.get("port") as string;
  const videoUrl = formData.get("videoUrl") as string;
  
  const connectionConfig = {
    host,
    port: parseInt(port),
    ws_port: parseInt(port), // Assuming same port for now or add field
    video_url: videoUrl
  };

  await db.insert(robots).values({
    robotName,
    robotType,
    backendType,
    connectionConfig,
    status: "offline",
  });

  revalidatePath("/admin");
}

export async function deleteRobot(id: string) {
  await db.delete(robots).where(eq(robots.id, id));
  revalidatePath("/admin");
}

export async function assignOperator(robotId: string, operatorClerkId: string) {
  await db.update(robots)
    .set({ assignedOperatorClerkId: operatorClerkId })
    .where(eq(robots.id, robotId));
  revalidatePath("/admin");
}

export async function getMyRobots(clerkId: string) {
  return await db.select().from(robots).where(eq(robots.assignedOperatorClerkId, clerkId));
}

export async function createSession(robotId: string, operatorClerkId: string) {
    const [session] = await db.insert(teleopSessions).values({
        robotId,
        operatorClerkId,
        startedAt: new Date(),
        status: 'active'
    }).returning();
    return session;
}

export async function endSession(sessionId: string) {
    await db.update(teleopSessions)
        .set({ 
            endedAt: new Date(), 
            status: 'completed' 
        })
        .where(eq(teleopSessions.id, sessionId));
}

export async function updateSessionMetrics(sessionId: string, metrics: any) {
    await db.update(teleopSessions)
        .set({
            commandsSent: metrics.commandsSent,
            averageLatencyMs: metrics.averageLatencyMs,
            maxLatencyMs: metrics.maxLatencyMs
        })
        .where(eq(teleopSessions.id, sessionId));
}
