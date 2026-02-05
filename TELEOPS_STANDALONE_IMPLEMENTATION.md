# TeleOps Platform - Standalone Implementation Guide

## Overview

Build TeleOps as a **self-contained platform** that teleoperators can use immediately, with future DeepReach integration points designed but not required for launch.

---

## Phase 1: Standalone TeleOps (No DeepReach Dependency)

### Goals
1. ✅ Teleoperators can register and log in via Clerk
2. ✅ Admins can manually configure robots via admin panel
3. ✅ Teleoperators can see assigned robots and control them
4. ✅ System tracks sessions and performance metrics
5. ✅ Ready for future DeepReach API integration

---

## Tech Stack

```yaml
Framework: Next.js 15 (App Router)
Database: PostgreSQL + Drizzle ORM
Auth: Clerk (separate instance from DeepReach for now)
Styling: Tailwind CSS 4
UI: Radix UI + shadcn/ui
Real-time: WebSocket (your existing teleop server)
Video: MJPEG streaming (your existing setup)
Deployment: Vercel (frontend) + AWS EC2 (teleop server)
Package Manager: pnpm
```

---

## Database Schema (TeleOps-Only)

### 1. **Robots Table**

```sql
CREATE TABLE tx_robots (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  
  -- Robot Info
  robot_name VARCHAR(255) NOT NULL,
  robot_type VARCHAR(100) NOT NULL,  -- 'franka_panda', 'ur5', 'so_arm100', 'custom'
  serial_number VARCHAR(255) UNIQUE,
  description TEXT,
  
  -- Connection Config
  backend_type VARCHAR(50) NOT NULL,  -- 'isaac', 'physical', 'mock'
  connection_config JSONB NOT NULL,   -- {host, port, ws_port, video_url}
  
  -- Example connection_config:
  -- {
  --   "host": "18.123.45.67",
  --   "http_port": 8000,
  --   "ws_port": 8000,
  --   "video_url": "/api/v1/video/mjpeg"
  -- }
  
  -- Assignment
  assigned_operator_clerk_id VARCHAR(255),  -- Clerk user ID
  
  -- Status
  status VARCHAR(50) DEFAULT 'offline',  -- 'online', 'offline', 'maintenance', 'in_use'
  last_seen_at TIMESTAMPTZ,
  
  -- Capabilities (for matching)
  capabilities TEXT[],  -- ['6dof_arm', 'gripper', 'vision']
  
  -- Metadata
  tags TEXT[],
  location VARCHAR(255),
  notes TEXT,
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW(),
  
  -- Future DeepReach integration
  external_factory_id VARCHAR(255),     -- DeepReach factory ID (nullable)
  external_robot_id VARCHAR(255),        -- DeepReach robot ID (nullable)
  sync_enabled BOOLEAN DEFAULT false     -- Enable sync with DeepReach
);

CREATE INDEX idx_robots_assigned_operator ON tx_robots(assigned_operator_clerk_id);
CREATE INDEX idx_robots_status ON tx_robots(status);
CREATE INDEX idx_robots_external_robot_id ON tx_robots(external_robot_id);
```

### 2. **Teleoperation Sessions Table**

```sql
CREATE TABLE tx_teleop_sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  
  -- Session Info
  operator_clerk_id VARCHAR(255) NOT NULL,
  operator_email VARCHAR(255),
  operator_name VARCHAR(255),
  robot_id UUID NOT NULL REFERENCES tx_robots(id),
  
  -- Session Lifecycle
  started_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  ended_at TIMESTAMPTZ,
  duration_seconds INT GENERATED ALWAYS AS (
    EXTRACT(EPOCH FROM (COALESCE(ended_at, NOW()) - started_at))
  ) STORED,
  
  -- Performance Metrics
  commands_sent INT DEFAULT 0,
  safety_violations INT DEFAULT 0,
  estop_count INT DEFAULT 0,
  average_latency_ms FLOAT,
  max_latency_ms FLOAT,
  connection_quality_score FLOAT,  -- 0-100
  
  -- Recording
  recording_s3_path VARCHAR(500),
  recording_size_mb FLOAT,
  
  -- Session Metadata
  session_metadata JSONB,  -- Additional logs, errors, notes
  
  -- Status
  status VARCHAR(50) DEFAULT 'active',  -- 'active', 'completed', 'aborted', 'error'
  end_reason VARCHAR(100),  -- 'normal', 'timeout', 'error', 'operator_disconnect'
  
  -- Future task tracking
  external_task_id VARCHAR(255),  -- DeepReach task ID (nullable)
  task_name VARCHAR(255),
  
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_sessions_operator ON tx_teleop_sessions(operator_clerk_id);
CREATE INDEX idx_sessions_robot ON tx_teleop_sessions(robot_id);
CREATE INDEX idx_sessions_started_at ON tx_teleop_sessions(started_at);
CREATE INDEX idx_sessions_status ON tx_teleop_sessions(status);
```

### 3. **Operator Profiles Table**

```sql
CREATE TABLE tx_operator_profiles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  
  -- Clerk Integration
  clerk_id VARCHAR(255) UNIQUE NOT NULL,
  email VARCHAR(255) NOT NULL,
  full_name VARCHAR(255),
  
  -- Profile Info
  tier VARCHAR(50) DEFAULT 'junior',  -- 'junior', 'senior', 'expert'
  certifications TEXT[],
  specializations TEXT[],  -- Robot types they can operate
  
  -- Work Stats (computed from sessions)
  total_sessions INT DEFAULT 0,
  total_hours FLOAT DEFAULT 0,
  average_session_rating FLOAT,
  
  -- Status
  is_active BOOLEAN DEFAULT true,
  last_login_at TIMESTAMPTZ,
  
  -- Preferences
  timezone VARCHAR(100) DEFAULT 'UTC',
  language VARCHAR(10) DEFAULT 'en',
  
  created_at TIMESTAMPTZ DEFAULT NOW(),
  updated_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_operator_profiles_clerk_id ON tx_operator_profiles(clerk_id);
CREATE INDEX idx_operator_profiles_is_active ON tx_operator_profiles(is_active);
```

### 4. **Admin Users Table**

```sql
CREATE TABLE tx_admin_users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  
  clerk_id VARCHAR(255) UNIQUE NOT NULL,
  email VARCHAR(255) NOT NULL,
  role VARCHAR(50) DEFAULT 'admin',  -- 'admin', 'super_admin'
  
  -- Permissions
  can_manage_robots BOOLEAN DEFAULT true,
  can_manage_operators BOOLEAN DEFAULT true,
  can_view_analytics BOOLEAN DEFAULT true,
  
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_admin_users_clerk_id ON tx_admin_users(clerk_id);
```

### 5. **Session Events Table** (for detailed logging)

```sql
CREATE TABLE tx_session_events (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  
  session_id UUID NOT NULL REFERENCES tx_teleop_sessions(id) ON DELETE CASCADE,
  
  -- Event Info
  event_type VARCHAR(100) NOT NULL,  -- 'command', 'safety_violation', 'estop', 'disconnect', 'error'
  event_data JSONB,
  
  -- Timing
  timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  
  created_at TIMESTAMPTZ DEFAULT NOW()
);

CREATE INDEX idx_session_events_session_id ON tx_session_events(session_id);
CREATE INDEX idx_session_events_timestamp ON tx_session_events(timestamp);
CREATE INDEX idx_session_events_type ON tx_session_events(event_type);
```

---

## Project Structure

```
teleops/
├── src/
│   ├── app/
│   │   ├── (auth)/
│   │   │   ├── sign-in/[[...sign-in]]/page.tsx
│   │   │   └── sign-up/[[...sign-up]]/page.tsx
│   │   │
│   │   ├── (operator)/                    # Teleoperator routes
│   │   │   ├── layout.tsx                 # Check operator role
│   │   │   ├── dashboard/page.tsx         # Main dashboard
│   │   │   ├── operate/[robotId]/page.tsx # Live control
│   │   │   ├── sessions/page.tsx          # Session history
│   │   │   └── performance/page.tsx       # Performance metrics
│   │   │
│   │   ├── (admin)/                       # Admin routes
│   │   │   ├── layout.tsx                 # Check admin role
│   │   │   ├── robots/
│   │   │   │   ├── page.tsx              # Robot list
│   │   │   │   ├── [id]/page.tsx         # Robot details
│   │   │   │   └── new/page.tsx          # Add new robot
│   │   │   ├── operators/
│   │   │   │   ├── page.tsx              # Operator list
│   │   │   │   └── [id]/page.tsx         # Operator details
│   │   │   ├── sessions/page.tsx          # All sessions
│   │   │   └── analytics/page.tsx         # System analytics
│   │   │
│   │   ├── api/
│   │   │   ├── robots/
│   │   │   │   ├── route.ts              # GET, POST
│   │   │   │   └── [id]/route.ts         # GET, PATCH, DELETE
│   │   │   ├── sessions/
│   │   │   │   ├── route.ts              # GET, POST (start)
│   │   │   │   └── [id]/
│   │   │   │       ├── route.ts          # GET session
│   │   │   │       └── end/route.ts      # POST (end session)
│   │   │   ├── operators/
│   │   │   │   ├── me/route.ts           # GET current operator
│   │   │   │   └── [id]/route.ts         # GET, PATCH operator
│   │   │   └── admin/
│   │   │       ├── robots/route.ts       # Admin robot management
│   │   │       └── operators/route.ts     # Admin operator management
│   │   │
│   │   ├── layout.tsx
│   │   └── page.tsx                       # Landing/redirect
│   │
│   ├── components/
│   │   ├── operator/
│   │   │   ├── DashboardOverview.tsx
│   │   │   ├── RobotCard.tsx
│   │   │   ├── SessionTimer.tsx
│   │   │   └── ControlInterface.tsx       # Reuse your web UI
│   │   ├── admin/
│   │   │   ├── RobotForm.tsx
│   │   │   ├── RobotTable.tsx
│   │   │   ├── OperatorTable.tsx
│   │   │   └── AssignmentModal.tsx
│   │   ├── shared/
│   │   │   ├── VideoStream.tsx
│   │   │   ├── RobotStatus.tsx
│   │   │   ├── PerformanceChart.tsx
│   │   │   └── SessionHistoryTable.tsx
│   │   └── ui/                            # shadcn components
│   │
│   ├── lib/
│   │   ├── db/
│   │   │   ├── index.ts                   # Drizzle client
│   │   │   └── schema.ts                  # Database schema
│   │   ├── teleop/
│   │   │   ├── client.ts                  # WebSocket client
│   │   │   └── session.ts                 # Session management
│   │   ├── auth/
│   │   │   └── roles.ts                   # Role checking utils
│   │   └── api/
│   │       └── client.ts                  # API client (for server components)
│   │
│   ├── hooks/
│   │   ├── useRobotConnection.ts
│   │   ├── useSession.ts
│   │   └── usePerformanceMetrics.ts
│   │
│   └── types/
│       ├── robot.ts
│       ├── session.ts
│       └── operator.ts
│
├── drizzle/
│   └── migrations/
├── public/
├── .env.example
├── drizzle.config.ts
├── middleware.ts                          # Clerk auth middleware
├── next.config.js
├── package.json
├── tsconfig.json
└── README.md
```

---

## Core Features Implementation

### 1. **Operator Dashboard** (`/dashboard`)

**Features**:
- List of assigned robots with status (online/offline/in_use)
- Quick connect buttons
- Active session indicator
- Recent session history (last 5)
- Performance summary (today's stats)

**Data Flow**:
```typescript
// Server Component
async function DashboardPage() {
  const { userId } = auth();
  
  // Get operator's assigned robots
  const robots = await db.query.robots.findMany({
    where: eq(schema.robots.assigned_operator_clerk_id, userId),
  });
  
  // Get active session
  const activeSession = await db.query.sessions.findFirst({
    where: and(
      eq(schema.sessions.operator_clerk_id, userId),
      eq(schema.sessions.status, 'active')
    ),
  });
  
  // Get recent sessions
  const recentSessions = await db.query.sessions.findMany({
    where: eq(schema.sessions.operator_clerk_id, userId),
    orderBy: desc(schema.sessions.started_at),
    limit: 5,
  });
  
  return (
    <DashboardOverview 
      robots={robots}
      activeSession={activeSession}
      recentSessions={recentSessions}
    />
  );
}
```

---

### 2. **Live Robot Control Interface** (`/operate/[robotId]`)

**Reuse existing web UI** with Next.js integration:

```typescript
// app/(operator)/operate/[robotId]/page.tsx
import { ControlInterface } from '@/components/operator/ControlInterface';

async function OperatePage({ params }: { params: { robotId: string } }) {
  const { userId } = auth();
  
  // Get robot details
  const robot = await db.query.robots.findFirst({
    where: eq(schema.robots.id, params.robotId),
  });
  
  if (!robot) {
    return <div>Robot not found</div>;
  }
  
  // Check assignment
  if (robot.assigned_operator_clerk_id !== userId) {
    return <div>You are not assigned to this robot</div>;
  }
  
  // Start or get active session
  const session = await getOrCreateActiveSession(userId, params.robotId);
  
  return (
    <ControlInterface 
      robot={robot}
      session={session}
      connectionConfig={robot.connection_config}
    />
  );
}
```

**ControlInterface Component**:
- Embed your existing `client/web/index.html` code
- Add session timer
- Add performance metrics display
- Add end session button
- Connect to teleop server WebSocket

```typescript
// components/operator/ControlInterface.tsx
'use client';

import { useEffect, useRef, useState } from 'react';
import { useRobotConnection } from '@/hooks/useRobotConnection';
import { VideoStream } from '@/components/shared/VideoStream';
import { SessionTimer } from './SessionTimer';

interface ControlInterfaceProps {
  robot: Robot;
  session: Session;
  connectionConfig: {
    host: string;
    http_port: number;
    ws_port: number;
    video_url: string;
  };
}

export function ControlInterface({ robot, session, connectionConfig }: ControlInterfaceProps) {
  const { 
    isConnected, 
    robotState, 
    sendCommand, 
    disconnect 
  } = useRobotConnection(connectionConfig);
  
  const [metrics, setMetrics] = useState({
    commandsSent: 0,
    safetyViolations: 0,
    avgLatency: 0
  });
  
  // Keyboard controls (same as your existing code)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Your existing WASD controls
      const cmd = mapKeyToCommand(e.key);
      if (cmd) {
        sendCommand(cmd);
        setMetrics(m => ({ ...m, commandsSent: m.commandsSent + 1 }));
      }
    };
    
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [sendCommand]);
  
  const handleEndSession = async () => {
    await fetch(`/api/sessions/${session.id}/end`, {
      method: 'POST',
      body: JSON.stringify({
        performance_metrics: metrics
      })
    });
    
    disconnect();
    router.push('/dashboard');
  };
  
  return (
    <div className="grid grid-cols-2 gap-4 p-4">
      {/* Left Panel - Video & 3D Viz */}
      <div className="space-y-4">
        <VideoStream 
          url={`http://${connectionConfig.host}:${connectionConfig.http_port}${connectionConfig.video_url}`}
        />
        
        <Canvas3D robotState={robotState} />
      </div>
      
      {/* Right Panel - Controls & Metrics */}
      <div className="space-y-4">
        <SessionTimer startTime={session.started_at} />
        
        <RobotStatusPanel robotState={robotState} />
        
        <PerformanceMetrics metrics={metrics} />
        
        <ControlPanel onCommand={sendCommand} />
        
        <button 
          onClick={handleEndSession}
          className="btn-danger"
        >
          End Session
        </button>
      </div>
    </div>
  );
}
```

---

### 3. **Admin Panel** (`/admin/robots`)

**Robot Management UI**:

```typescript
// app/(admin)/robots/page.tsx
async function AdminRobotsPage() {
  const { userId } = auth();
  
  // Check if user is admin
  const admin = await db.query.adminUsers.findFirst({
    where: eq(schema.adminUsers.clerk_id, userId),
  });
  
  if (!admin) {
    redirect('/dashboard');
  }
  
  const robots = await db.query.robots.findMany({
    orderBy: desc(schema.robots.created_at),
  });
  
  const operators = await db.query.operatorProfiles.findMany({
    where: eq(schema.operatorProfiles.is_active, true),
  });
  
  return (
    <div>
      <div className="flex justify-between items-center mb-6">
        <h1 className="text-2xl font-bold">Robot Fleet Management</h1>
        <Link href="/admin/robots/new">
          <button className="btn-primary">Add Robot</button>
        </Link>
      </div>
      
      <RobotTable 
        robots={robots} 
        operators={operators}
      />
    </div>
  );
}
```

**Add Robot Form** (`/admin/robots/new`):

```typescript
// app/(admin)/robots/new/page.tsx
'use client';

import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';

const robotSchema = z.object({
  robot_name: z.string().min(1),
  robot_type: z.enum(['franka_panda', 'ur5', 'so_arm100', 'custom']),
  serial_number: z.string().optional(),
  backend_type: z.enum(['isaac', 'physical', 'mock']),
  connection_config: z.object({
    host: z.string(),
    http_port: z.number(),
    ws_port: z.number(),
    video_url: z.string(),
  }),
  location: z.string().optional(),
  capabilities: z.array(z.string()).optional(),
});

export function AddRobotPage() {
  const form = useForm({
    resolver: zodResolver(robotSchema),
    defaultValues: {
      robot_name: '',
      robot_type: 'franka_panda',
      backend_type: 'isaac',
      connection_config: {
        host: '',
        http_port: 8000,
        ws_port: 8000,
        video_url: '/api/v1/video/mjpeg',
      },
    }
  });
  
  const onSubmit = async (data: z.infer<typeof robotSchema>) => {
    const res = await fetch('/api/admin/robots', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data),
    });
    
    if (res.ok) {
      router.push('/admin/robots');
    }
  };
  
  return (
    <form onSubmit={form.handleSubmit(onSubmit)} className="max-w-2xl space-y-6">
      <h1 className="text-2xl font-bold">Add New Robot</h1>
      
      <div>
        <label>Robot Name</label>
        <input {...form.register('robot_name')} />
      </div>
      
      <div>
        <label>Robot Type</label>
        <select {...form.register('robot_type')}>
          <option value="franka_panda">Franka Panda</option>
          <option value="ur5">Universal Robots UR5</option>
          <option value="so_arm100">SO-ARM100</option>
          <option value="custom">Custom</option>
        </select>
      </div>
      
      <div>
        <label>Backend Type</label>
        <select {...form.register('backend_type')}>
          <option value="isaac">Isaac Sim</option>
          <option value="physical">Physical Robot</option>
          <option value="mock">Mock (Testing)</option>
        </select>
      </div>
      
      <fieldset className="border p-4 rounded">
        <legend className="font-semibold">Connection Config</legend>
        
        <div className="grid grid-cols-2 gap-4">
          <div>
            <label>Host (IP or Domain)</label>
            <input {...form.register('connection_config.host')} />
          </div>
          
          <div>
            <label>HTTP Port</label>
            <input type="number" {...form.register('connection_config.http_port', { valueAsNumber: true })} />
          </div>
          
          <div>
            <label>WebSocket Port</label>
            <input type="number" {...form.register('connection_config.ws_port', { valueAsNumber: true })} />
          </div>
          
          <div>
            <label>Video URL Path</label>
            <input {...form.register('connection_config.video_url')} />
          </div>
        </div>
      </fieldset>
      
      <div>
        <label>Location (Optional)</label>
        <input {...form.register('location')} placeholder="Factory A, Beijing" />
      </div>
      
      <button type="submit" className="btn-primary">
        Add Robot
      </button>
    </form>
  );
}
```

---

### 4. **Assign Robot to Operator**

```typescript
// app/(admin)/robots/[id]/page.tsx
async function RobotDetailPage({ params }: { params: { id: string } }) {
  const robot = await db.query.robots.findFirst({
    where: eq(schema.robots.id, params.id),
  });
  
  const operators = await db.query.operatorProfiles.findMany({
    where: eq(schema.operatorProfiles.is_active, true),
  });
  
  const sessions = await db.query.sessions.findMany({
    where: eq(schema.sessions.robot_id, params.id),
    orderBy: desc(schema.sessions.started_at),
    limit: 10,
  });
  
  return (
    <div>
      <h1>{robot.robot_name}</h1>
      
      <RobotStatusCard robot={robot} />
      
      <AssignOperatorSection 
        robot={robot} 
        operators={operators}
      />
      
      <SessionHistoryTable sessions={sessions} />
    </div>
  );
}
```

**Assignment API**:

```typescript
// app/api/admin/robots/[id]/assign/route.ts
export async function POST(
  req: Request,
  { params }: { params: { id: string } }
) {
  const { userId } = auth();
  
  // Check admin permission
  const admin = await db.query.adminUsers.findFirst({
    where: eq(schema.adminUsers.clerk_id, userId),
  });
  
  if (!admin) {
    return Response.json({ error: 'Unauthorized' }, { status: 401 });
  }
  
  const { operator_clerk_id } = await req.json();
  
  // Update robot assignment
  await db.update(schema.robots)
    .set({ 
      assigned_operator_clerk_id: operator_clerk_id,
      updated_at: new Date(),
    })
    .where(eq(schema.robots.id, params.id));
  
  return Response.json({ success: true });
}
```

---

## Clerk Configuration

### 1. **User Roles Setup**

Add custom metadata to Clerk users:

```typescript
// lib/auth/roles.ts
export type UserRole = 'operator' | 'admin';

export async function getUserRole(userId: string): Promise<UserRole | null> {
  const clerkClient = await import('@clerk/nextjs/server').then(m => m.clerkClient);
  const user = await clerkClient.users.getUser(userId);
  
  // Check if admin
  const isAdmin = await db.query.adminUsers.findFirst({
    where: eq(schema.adminUsers.clerk_id, userId),
  });
  
  if (isAdmin) return 'admin';
  
  // Check if operator
  const isOperator = await db.query.operatorProfiles.findFirst({
    where: eq(schema.operatorProfiles.clerk_id, userId),
  });
  
  if (isOperator) return 'operator';
  
  return null;
}

export async function requireRole(role: UserRole) {
  const { userId } = auth();
  
  if (!userId) {
    redirect('/sign-in');
  }
  
  const userRole = await getUserRole(userId);
  
  if (userRole !== role && userRole !== 'admin') {
    redirect('/dashboard');
  }
}
```

### 2. **Middleware Configuration**

```typescript
// middleware.ts
import { authMiddleware } from "@clerk/nextjs";
import { NextResponse } from "next/server";

export default authMiddleware({
  publicRoutes: ["/", "/sign-in(.*)", "/sign-up(.*)"],
  
  afterAuth(auth, req) {
    // Not signed in -> redirect to sign-in
    if (!auth.userId && !auth.isPublicRoute) {
      const signInUrl = new URL('/sign-in', req.url);
      signInUrl.searchParams.set('redirect_url', req.url);
      return NextResponse.redirect(signInUrl);
    }
    
    // Signed in but on home page -> redirect to dashboard
    if (auth.userId && req.nextUrl.pathname === '/') {
      return NextResponse.redirect(new URL('/dashboard', req.url));
    }
    
    return NextResponse.next();
  }
});

export const config = {
  matcher: ['/((?!.+\\.[\\w]+$|_next).*)', '/', '/(api|trpc)(.*)'],
};
```

### 3. **New User Onboarding**

Auto-create operator profile when user signs up:

```typescript
// app/api/webhooks/clerk/route.ts
import { Webhook } from 'svix';
import { headers } from 'next/headers';

export async function POST(req: Request) {
  const WEBHOOK_SECRET = process.env.CLERK_WEBHOOK_SECRET;
  
  if (!WEBHOOK_SECRET) {
    throw new Error('Missing CLERK_WEBHOOK_SECRET');
  }
  
  const headerPayload = headers();
  const svix_id = headerPayload.get("svix-id");
  const svix_timestamp = headerPayload.get("svix-timestamp");
  const svix_signature = headerPayload.get("svix-signature");
  
  if (!svix_id || !svix_timestamp || !svix_signature) {
    return new Response('Error', { status: 400 });
  }
  
  const payload = await req.json();
  const body = JSON.stringify(payload);
  
  const wh = new Webhook(WEBHOOK_SECRET);
  let evt;
  
  try {
    evt = wh.verify(body, {
      "svix-id": svix_id,
      "svix-timestamp": svix_timestamp,
      "svix-signature": svix_signature,
    });
  } catch (err) {
    return new Response('Error', { status: 400 });
  }
  
  const eventType = evt.type;
  
  if (eventType === 'user.created') {
    const { id, email_addresses, first_name, last_name } = evt.data;
    
    // Create operator profile
    await db.insert(schema.operatorProfiles).values({
      clerk_id: id,
      email: email_addresses[0].email_address,
      full_name: `${first_name} ${last_name}`,
      tier: 'junior',
      is_active: true,
    });
  }
  
  return new Response('', { status: 200 });
}
```

---

## Session Management

### WebSocket Connection with Session Tracking

```typescript
// hooks/useRobotConnection.ts
'use client';

import { useEffect, useState, useCallback } from 'react';

interface ConnectionConfig {
  host: string;
  http_port: number;
  ws_port: number;
  video_url: string;
}

export function useRobotConnection(config: ConnectionConfig) {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [robotState, setRobotState] = useState<any>(null);
  const [metrics, setMetrics] = useState({
    commandsSent: 0,
    latencyMs: 0,
  });
  
  useEffect(() => {
    // Connect to WebSocket
    const wsUrl = `ws://${config.host}:${config.ws_port}/api/v1/ws`;
    const socket = new WebSocket(wsUrl);
    
    socket.onopen = () => {
      console.log('WebSocket connected');
      setIsConnected(true);
    };
    
    socket.onmessage = (event) => {
      const msg = JSON.parse(event.data);
      
      if (msg.type === 'state') {
        setRobotState(msg.robot);
      }
      
      if (msg.type === 'ack') {
        // Update latency
        const latency = Date.now() - msg.timestamp;
        setMetrics(m => ({
          ...m,
          latencyMs: latency,
        }));
      }
    };
    
    socket.onclose = () => {
      console.log('WebSocket disconnected');
      setIsConnected(false);
    };
    
    setWs(socket);
    
    return () => {
      socket.close();
    };
  }, [config]);
  
  const sendCommand = useCallback((command: any) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'delta',
        payload: {
          ...command,
          timestamp: Date.now(),
        }
      }));
      
      setMetrics(m => ({
        ...m,
        commandsSent: m.commandsSent + 1,
      }));
    }
  }, [ws]);
  
  const disconnect = useCallback(() => {
    if (ws) {
      ws.close();
    }
  }, [ws]);
  
  return {
    isConnected,
    robotState,
    metrics,
    sendCommand,
    disconnect,
  };
}
```

---

## Environment Setup

### .env.example

```bash
# Database
DATABASE_URL="postgresql://user:password@localhost:5432/teleops"

# Clerk
NEXT_PUBLIC_CLERK_PUBLISHABLE_KEY=pk_test_xxxxx
CLERK_SECRET_KEY=sk_test_xxxxx
CLERK_WEBHOOK_SECRET=whsec_xxxxx

# App Config
NEXT_PUBLIC_APP_URL=http://localhost:3000

# AWS (for session recordings - optional)
AWS_ACCESS_KEY_ID=
AWS_SECRET_ACCESS_KEY=
AWS_REGION=us-east-1
AWS_S3_BUCKET=teleops-recordings
```

---

## Deployment Plan

### Phase 1: Local Development (Week 1-2)

1. ✅ Set up Next.js project with Drizzle + Clerk
2. ✅ Create database schema
3. ✅ Build admin panel (add robots, assign operators)
4. ✅ Build operator dashboard (view assigned robots)

### Phase 2: Robot Control Integration (Week 3-4)

1. ✅ Integrate existing web UI into Next.js
2. ✅ Connect to existing teleop server WebSocket
3. ✅ Add session start/end tracking
4. ✅ Add performance metrics collection

### Phase 3: Session & Analytics (Week 5-6)

1. ✅ Build session history page
2. ✅ Build performance dashboard
3. ✅ Add session event logging
4. ✅ (Optional) Add session recording to S3

### Phase 4: Polish & Deploy (Week 7-8)

1. ✅ UI/UX improvements
2. ✅ Add real-time status updates
3. ✅ Deploy to Vercel
4. ✅ Test end-to-end workflow

---

## Future DeepReach Integration Points

When DeepReach APIs are ready:

### 1. **Sync Robot Data**

Add background job to sync robots from DeepReach:

```typescript
// app/api/cron/sync-robots/route.ts
export async function GET(req: Request) {
  // Verify cron secret
  if (req.headers.get('authorization') !== `Bearer ${process.env.CRON_SECRET}`) {
    return Response.json({ error: 'Unauthorized' }, { status: 401 });
  }
  
  // Fetch robots from DeepReach API
  const deepreachRobots = await fetch('https://deepreach.ai/api/v1/robots', {
    headers: {
      'Authorization': `Bearer ${process.env.DEEPREACH_API_KEY}`,
    },
  }).then(r => r.json());
  
  // Upsert robots into local DB
  for (const dr_robot of deepreachRobots) {
    await db.insert(schema.robots)
      .values({
        external_robot_id: dr_robot.id,
        external_factory_id: dr_robot.factory_id,
        robot_name: dr_robot.robot_name,
        robot_type: dr_robot.robot_type,
        connection_config: dr_robot.backend_config,
        assigned_operator_clerk_id: dr_robot.assigned_operator_id,
        sync_enabled: true,
      })
      .onConflictDoUpdate({
        target: schema.robots.external_robot_id,
        set: {
          robot_name: dr_robot.robot_name,
          connection_config: dr_robot.backend_config,
          assigned_operator_clerk_id: dr_robot.assigned_operator_id,
          updated_at: new Date(),
        }
      });
  }
  
  return Response.json({ success: true });
}
```

### 2. **Push Session Data to DeepReach**

When session ends, send metrics to DeepReach:

```typescript
// lib/deepreach/sync.ts
export async function syncSessionToDeepReach(session: Session) {
  if (!session.external_task_id) {
    return; // Not linked to DeepReach task
  }
  
  await fetch('https://deepreach.ai/api/v1/teleop-sessions', {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${process.env.DEEPREACH_API_KEY}`,
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      task_id: session.external_task_id,
      operator_clerk_id: session.operator_clerk_id,
      robot_id: session.external_robot_id,
      started_at: session.started_at,
      ended_at: session.ended_at,
      duration_seconds: session.duration_seconds,
      performance_metrics: {
        commands_sent: session.commands_sent,
        safety_violations: session.safety_violations,
        average_latency_ms: session.average_latency_ms,
      }
    }),
  });
}
```

---

## Quick Start Commands

```bash
# 1. Clone and setup
git clone <your-teleops-repo>
cd teleops
pnpm install

# 2. Setup database
cp .env.example .env.local
# Edit .env.local with your credentials

# 3. Run database migrations
pnpm db:push

# 4. Seed initial admin user (run once)
pnpm tsx scripts/seed-admin.ts

# 5. Start dev server
pnpm dev

# 6. Open http://localhost:3000
```

---

## Summary

This design gives you a **fully functional standalone TeleOps platform** that:

1. ✅ Works independently without DeepReach code changes
2. ✅ Reuses your existing teleop server infrastructure
3. ✅ Provides admin panel for robot/operator management
4. ✅ Tracks sessions and performance metrics
5. ✅ Ready for future DeepReach integration via APIs
6. ✅ Uses same tech stack as DeepReach (easy for your team)

**Start with Phase 1-2** to get basic functionality working, then iterate!
