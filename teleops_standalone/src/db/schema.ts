import { pgTable, uuid, varchar, text, jsonb, timestamp, boolean, integer, doublePrecision, index } from "drizzle-orm/pg-core";
import { sql } from "drizzle-orm";

export const robots = pgTable("tx_robots", {
  id: uuid("id").defaultRandom().primaryKey(),
  robotName: varchar("robot_name", { length: 255 }).notNull(),
  robotType: varchar("robot_type", { length: 100 }).notNull(),
  serialNumber: varchar("serial_number", { length: 255 }).unique(),
  description: text("description"),
  backendType: varchar("backend_type", { length: 50 }).notNull(),
  connectionConfig: jsonb("connection_config").notNull(),
  assignedOperatorClerkId: varchar("assigned_operator_clerk_id", { length: 255 }),
  status: varchar("status", { length: 50 }).default("offline"),
  lastSeenAt: timestamp("last_seen_at", { withTimezone: true }),
  capabilities: text("capabilities").array(),
  tags: text("tags").array(),
  location: varchar("location", { length: 255 }),
  notes: text("notes"),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow(),
  updatedAt: timestamp("updated_at", { withTimezone: true }).defaultNow(),
  externalFactoryId: varchar("external_factory_id", { length: 255 }),
  externalRobotId: varchar("external_robot_id", { length: 255 }),
  syncEnabled: boolean("sync_enabled").default(false),
}, (table) => {
  return {
    assignedOperatorIdx: index("idx_robots_assigned_operator").on(table.assignedOperatorClerkId),
    statusIdx: index("idx_robots_status").on(table.status),
    externalRobotIdIdx: index("idx_robots_external_robot_id").on(table.externalRobotId),
  };
});

export const teleopSessions = pgTable("tx_teleop_sessions", {
  id: uuid("id").defaultRandom().primaryKey(),
  operatorClerkId: varchar("operator_clerk_id", { length: 255 }).notNull(),
  operatorEmail: varchar("operator_email", { length: 255 }),
  operatorName: varchar("operator_name", { length: 255 }),
  robotId: uuid("robot_id").notNull().references(() => robots.id),
  startedAt: timestamp("started_at", { withTimezone: true }).defaultNow().notNull(),
  endedAt: timestamp("ended_at", { withTimezone: true }),
  commandsSent: integer("commands_sent").default(0),
  safetyViolations: integer("safety_violations").default(0),
  estopCount: integer("estop_count").default(0),
  averageLatencyMs: doublePrecision("average_latency_ms"),
  maxLatencyMs: doublePrecision("max_latency_ms"),
  connectionQualityScore: doublePrecision("connection_quality_score"),
  recordingS3Path: varchar("recording_s3_path", { length: 500 }),
  recordingSizeMb: doublePrecision("recording_size_mb"),
  sessionMetadata: jsonb("session_metadata"),
  status: varchar("status", { length: 50 }).default("active"),
  endReason: varchar("end_reason", { length: 100 }),
  externalTaskId: varchar("external_task_id", { length: 255 }),
  taskName: varchar("task_name", { length: 255 }),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow(),
  updatedAt: timestamp("updated_at", { withTimezone: true }).defaultNow(),
}, (table) => {
  return {
    operatorIdx: index("idx_sessions_operator").on(table.operatorClerkId),
    robotIdx: index("idx_sessions_robot").on(table.robotId),
    startedAtIdx: index("idx_sessions_started_at").on(table.startedAt),
    statusIdx: index("idx_sessions_status").on(table.status),
  };
});

export const operatorProfiles = pgTable("tx_operator_profiles", {
  id: uuid("id").defaultRandom().primaryKey(),
  clerkId: varchar("clerk_id", { length: 255 }).unique().notNull(),
  email: varchar("email", { length: 255 }).notNull(),
  fullName: varchar("full_name", { length: 255 }),
  tier: varchar("tier", { length: 50 }).default("junior"),
  certifications: text("certifications").array(),
  specializations: text("specializations").array(),
  totalSessions: integer("total_sessions").default(0),
  totalHours: doublePrecision("total_hours").default(0),
  averageSessionRating: doublePrecision("average_session_rating"),
  isActive: boolean("is_active").default(true),
  lastLoginAt: timestamp("last_login_at", { withTimezone: true }),
  timezone: varchar("timezone", { length: 100 }).default("UTC"),
  language: varchar("language", { length: 10 }).default("en"),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow(),
  updatedAt: timestamp("updated_at", { withTimezone: true }).defaultNow(),
}, (table) => {
  return {
    clerkIdIdx: index("idx_operator_profiles_clerk_id").on(table.clerkId),
    isActiveIdx: index("idx_operator_profiles_is_active").on(table.isActive),
  };
});

export const adminUsers = pgTable("tx_admin_users", {
  id: uuid("id").defaultRandom().primaryKey(),
  clerkId: varchar("clerk_id", { length: 255 }).unique().notNull(),
  email: varchar("email", { length: 255 }).notNull(),
  role: varchar("role", { length: 50 }).default("admin"),
  canManageRobots: boolean("can_manage_robots").default(true),
  canManageOperators: boolean("can_manage_operators").default(true),
  canViewAnalytics: boolean("can_view_analytics").default(true),
  createdAt: timestamp("created_at", { withTimezone: true }).defaultNow(),
}, (table) => {
  return {
    clerkIdIdx: index("idx_admin_users_clerk_id").on(table.clerkId),
  };
});
