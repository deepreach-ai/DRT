'use client'

import { useEffect, useRef, useState, useCallback } from 'react';
import { createSession, endSession, updateSessionMetrics } from '@/app/actions';
import { useRouter } from 'next/navigation';
import Link from 'next/link';

// Types
type Robot = {
  id: string;
  robotName: string;
  robotType: string;
  backendType: string;
  connectionConfig: any;
};

type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'error';

export default function ControlRoom({ robot, userId }: { robot: Robot; userId: string }) {
  const router = useRouter();
  const [status, setStatus] = useState<ConnectionStatus>('disconnected');
  const [latency, setLatency] = useState<number>(0);
  const [commandCount, setCommandCount] = useState<number>(0);
  const [safetyActive, setSafetyActive] = useState<boolean>(false);
  const [controlMode, setControlMode] = useState<'position' | 'orientation'>('position');
  const [sessionId, setSessionId] = useState<string | null>(null);
  
  // Refs for WebSocket and state that needs to be accessed in loops/callbacks
  const wsRef = useRef<WebSocket | null>(null);
  const activeKeysRef = useRef<Set<string>>(new Set());
  const commandLoopRef = useRef<NodeJS.Timeout | null>(null);
  const metricsLoopRef = useRef<NodeJS.Timeout | null>(null);
  const statsRef = useRef({
    commandsSent: 0,
    latencies: [] as number[],
    lastCommandTime: 0
  });

  // Configuration
  const CONFIG = {
    positionIncrement: 0.01, // meters
    orientationIncrement: 0.03, // radians
    sendFrequency: 20, // Hz
    metricsInterval: 5000, // ms
  };

  const keyMappings: Record<string, { axis: string; value: number }> = {
    'w': { axis: 'dz', value: 1 },      // Up
    's': { axis: 'dz', value: -1 },     // Down
    'a': { axis: 'dx', value: -1 },     // Left
    'd': { axis: 'dx', value: 1 },      // Right
    'q': { axis: 'dy', value: 1 },      // Forward
    'e': { axis: 'dy', value: -1 },     // Backward
    'i': { axis: 'dpitch', value: -1 }, // Pitch down
    'k': { axis: 'dpitch', value: 1 },  // Pitch up
    'j': { axis: 'dyaw', value: 1 },    // Yaw left
    'l': { axis: 'dyaw', value: -1 },   // Yaw right
    'u': { axis: 'droll', value: -1 },  // Roll left
    'o': { axis: 'droll', value: 1 },   // Roll right
  };

  // Initialize Session
  useEffect(() => {
    const initSession = async () => {
      try {
        const session = await createSession(robot.id, userId);
        setSessionId(session.id);
        connectWebSocket();
      } catch (error) {
        console.error("Failed to create session:", error);
        setStatus('error');
      }
    };

    initSession();

    return () => {
      cleanup();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Cleanup function
  const cleanup = useCallback(async () => {
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    
    if (commandLoopRef.current) {
      clearInterval(commandLoopRef.current);
      commandLoopRef.current = null;
    }

    if (metricsLoopRef.current) {
      clearInterval(metricsLoopRef.current);
      metricsLoopRef.current = null;
    }

    if (sessionId) {
      // Sync final metrics
      await updateMetrics();
      await endSession(sessionId);
    }
  }, [sessionId]);

  // Update Metrics
  const updateMetrics = async () => {
    if (!sessionId) return;
    
    const latencies = statsRef.current.latencies;
    const avgLatency = latencies.length > 0 
      ? latencies.reduce((a, b) => a + b, 0) / latencies.length 
      : 0;
    const maxLatency = latencies.length > 0 ? Math.max(...latencies) : 0;

    await updateSessionMetrics(sessionId, {
      commandsSent: statsRef.current.commandsSent,
      averageLatencyMs: avgLatency,
      maxLatencyMs: maxLatency
    });
    
    // Reset latencies buffer to keep it rolling or just clear it? 
    // For now, let's keep accumulating or maybe just keep last N?
    // Let's clear it to get "current" average
    statsRef.current.latencies = [];
  };

  // WebSocket Connection
  const connectWebSocket = () => {
    const config = robot.connectionConfig;
    // Construct WS URL - ensure we use the correct protocol/host/port
    // Assuming config has host and ws_port. 
    // If running in browser, accessing localhost might be tricky if backend is in container?
    // But for standalone, we assume backend is reachable.
    
    // Use window.location.hostname if config.host is 'localhost' to handle remote access better?
    // Or just trust config.
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${config.host}:${config.ws_port || 8000}/ws/v1/teleop`;
    
    console.log(`Connecting to ${wsUrl}`);
    setStatus('connecting');

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('Connected');
        setStatus('connected');
        setSafetyActive(false); // Default to off, let user enable? Or auto-enable?
        // Original code auto-enabled safety after 500ms
        setTimeout(() => setSafetyActive(true), 500);
        
        startLoops();
      };

      ws.onclose = () => {
        console.log('Disconnected');
        setStatus('disconnected');
        stopLoops();
      };

      ws.onerror = (err) => {
        console.error('WS Error', err);
        setStatus('error');
      };

      ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        handleServerMessage(data);
      };

    } catch (err) {
      console.error("WS Connection Failed", err);
      setStatus('error');
    }
  };

  const startLoops = () => {
    if (commandLoopRef.current) clearInterval(commandLoopRef.current);
    commandLoopRef.current = setInterval(sendCommand, 1000 / CONFIG.sendFrequency);

    if (metricsLoopRef.current) clearInterval(metricsLoopRef.current);
    metricsLoopRef.current = setInterval(updateMetrics, CONFIG.metricsInterval);
  };

  const stopLoops = () => {
    if (commandLoopRef.current) clearInterval(commandLoopRef.current);
    if (metricsLoopRef.current) clearInterval(metricsLoopRef.current);
  };

  const handleServerMessage = (data: any) => {
    if (data.type === 'status') {
      // Handle status update
    } else if (data.type === 'pong') {
       const rtt = Date.now() - data.timestamp;
       setLatency(rtt);
       statsRef.current.latencies.push(rtt);
    }
  };

  const sendCommand = () => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) return;
    if (activeKeysRef.current.size === 0) return;

    const cmd: any = {
      type: 'control',
      timestamp: Date.now(),
      mode: controlMode,
      command: {
        x: 0, y: 0, z: 0,
        roll: 0, pitch: 0, yaw: 0,
        gripper: 0
      }
    };

    activeKeysRef.current.forEach(key => {
      const mapping = keyMappings[key];
      if (mapping) {
        const increment = controlMode === 'position' ? CONFIG.positionIncrement : CONFIG.orientationIncrement;
        // Map axes to command fields
        // Note: The mapping in original js was:
        // dz -> z (up/down)
        // dx -> x (left/right) -> wait, standard coords: X forward, Y left, Z up?
        // Let's follow the JS logic:
        // w/s -> dz -> z
        // a/d -> dx -> x
        // q/e -> dy -> y
        
        // Wait, original JS:
        // w (up) -> dz +1
        // s (down) -> dz -1
        // a (left) -> dx -1
        // d (right) -> dx 1
        // q (fwd) -> dy 1
        // e (back) -> dy -1
        
        switch(mapping.axis) {
          case 'dx': cmd.command.x += mapping.value * increment; break;
          case 'dy': cmd.command.y += mapping.value * increment; break;
          case 'dz': cmd.command.z += mapping.value * increment; break;
          case 'droll': cmd.command.roll += mapping.value * increment; break;
          case 'dpitch': cmd.command.pitch += mapping.value * increment; break;
          case 'dyaw': cmd.command.yaw += mapping.value * increment; break;
        }
      }
    });

    wsRef.current.send(JSON.stringify(cmd));
    
    setCommandCount(prev => prev + 1);
    statsRef.current.commandsSent++;
    statsRef.current.lastCommandTime = Date.now();
  };

  // Input Handling
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      
      // Special keys
      if (key === 'r') {
         // Reset?
         return;
      }
      if (key === '1') {
         setSafetyActive(prev => !prev);
         // Send safety toggle?
         return;
      }
      if (key === 'm') {
         setControlMode(prev => prev === 'position' ? 'orientation' : 'position');
         return;
      }

      if (keyMappings[key]) {
        activeKeysRef.current.add(key);
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      const key = e.key.toLowerCase();
      if (keyMappings[key]) {
        activeKeysRef.current.delete(key);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [controlMode]); // Re-bind if necessary, or just use ref for mode if needed inside loops

  // Video URL construction
  const videoUrl = robot.connectionConfig.video_url.startsWith('http') 
    ? robot.connectionConfig.video_url 
    : `http://${robot.connectionConfig.host}:${robot.connectionConfig.port}${robot.connectionConfig.video_url}`;

  return (
    <div className="flex h-full flex-col">
      {/* Header */}
      <div className="bg-gray-800 p-4 flex justify-between items-center shadow-md">
        <div className="flex items-center gap-4">
          <Link href="/dashboard" className="text-gray-400 hover:text-white">
            &larr; Exit
          </Link>
          <h1 className="text-xl font-bold">{robot.robotName}</h1>
          <span className={`px-2 py-0.5 rounded text-xs uppercase font-bold ${
            status === 'connected' ? 'bg-green-500 text-white' : 
            status === 'connecting' ? 'bg-yellow-500 text-black' : 
            'bg-red-500 text-white'
          }`}>
            {status}
          </span>
        </div>
        
        <div className="flex items-center gap-6 text-sm">
          <div className="flex flex-col items-center">
            <span className="text-gray-400 text-xs">Latency</span>
            <span className="font-mono font-bold">{latency} ms</span>
          </div>
          <div className="flex flex-col items-center">
             <span className="text-gray-400 text-xs">Commands</span>
             <span className="font-mono font-bold">{commandCount}</span>
          </div>
          <div className="flex flex-col items-center">
             <span className="text-gray-400 text-xs">Mode</span>
             <span className="font-bold text-blue-400 uppercase">{controlMode}</span>
          </div>
          <div className={`flex items-center gap-2 px-3 py-1 rounded border ${
            safetyActive ? 'border-green-500 text-green-500' : 'border-red-500 text-red-500'
          }`}>
             <div className={`w-3 h-3 rounded-full ${safetyActive ? 'bg-green-500' : 'bg-red-500'}`}></div>
             <span className="font-bold">SAFETY {safetyActive ? 'ON' : 'OFF'}</span>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex overflow-hidden">
        {/* Video Feed */}
        <div className="flex-1 bg-black relative flex items-center justify-center">
          {status === 'connected' ? (
             // eslint-disable-next-line @next/next/no-img-element
             <img 
               src={videoUrl} 
               alt="Robot Stream" 
               className="max-w-full max-h-full object-contain"
             />
          ) : (
            <div className="text-gray-500 flex flex-col items-center">
              <p>Waiting for connection...</p>
              <p className="text-xs mt-2">{videoUrl}</p>
            </div>
          )}
          
          {/* Overlay Controls / HUD */}
          <div className="absolute bottom-4 left-4 text-white/50 text-xs pointer-events-none">
             <p>WASD: Move X/Z</p>
             <p>Q/E: Move Y</p>
             <p>I/K: Pitch</p>
             <p>J/L: Yaw</p>
             <p>U/O: Roll</p>
             <p>M: Toggle Mode</p>
          </div>
        </div>

        {/* Sidebar Controls (Optional) */}
        <div className="w-64 bg-gray-900 border-l border-gray-800 p-4 flex flex-col gap-4 overflow-y-auto">
           <h3 className="font-bold text-gray-400 uppercase text-xs">Session Info</h3>
           <div className="text-sm">
             <p className="flex justify-between"><span>ID:</span> <span className="font-mono text-xs">{sessionId?.slice(0,8)}...</span></p>
             <p className="flex justify-between"><span>Started:</span> <span>{new Date().toLocaleTimeString()}</span></p>
           </div>
           
           <hr className="border-gray-800" />
           
           <h3 className="font-bold text-gray-400 uppercase text-xs">Controls</h3>
           <div className="grid grid-cols-2 gap-2">
              <button 
                className={`p-2 rounded text-sm font-bold ${controlMode === 'position' ? 'bg-blue-600' : 'bg-gray-800'}`}
                onClick={() => setControlMode('position')}
              >
                Position
              </button>
              <button 
                className={`p-2 rounded text-sm font-bold ${controlMode === 'orientation' ? 'bg-blue-600' : 'bg-gray-800'}`}
                onClick={() => setControlMode('orientation')}
              >
                Orientation
              </button>
           </div>
           
           <button 
              className={`w-full p-3 rounded font-bold mt-4 ${safetyActive ? 'bg-green-600 hover:bg-green-700' : 'bg-red-600 hover:bg-red-700'}`}
              onClick={() => setSafetyActive(!safetyActive)}
           >
             {safetyActive ? 'SAFETY ENABLED' : 'SAFETY DISABLED'}
           </button>

           <div className="mt-auto">
             <button 
               onClick={() => router.push('/dashboard')}
               className="w-full bg-red-900/50 hover:bg-red-900 text-red-200 p-2 rounded border border-red-900"
             >
               End Session
             </button>
           </div>
        </div>
      </div>
    </div>
  );
}
