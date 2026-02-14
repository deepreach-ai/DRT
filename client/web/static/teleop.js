// Teleoperation Web Client
class TeleopClient {
    constructor() {
        // Configuration - get server URL from page or default to localhost
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.hostname || 'localhost';
        const port = window.location.port;
        
        // Handle port logic: use port if present, otherwise default to 8000 only if localhost
        // For ngrok (https on 443), port is empty string, so we shouldn't append :8000
        let portStr = port ? `:${port}` : '';
        if (!port && host === 'localhost') {
             portStr = ':8000';
        }
        
        this.wsUrl = `${protocol}//${host}${portStr}/ws/v1/teleop`;
        this.apiUrl = `${window.location.protocol}//${host}${portStr}/api/v1`;
        this.token = null;
        
        // WebSocket
        this.ws = null;
        this.connected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        
        // Control state
        this.controlMode = 'position'; // 'position' or 'orientation'
        this.activeKeys = new Set();
        
        // Control increments (per command)
        this.positionIncrement = 0.01; // meters
        this.orientationIncrement = 0.03; // radians
        
        // Command sending
        this.sendInterval = null;
        this.sendFrequency = 50; // Hz (Increased for lower latency)
        
        // Statistics
        this.commandCount = 0;
        this.startTime = null;
        this.lastCommandTime = 0;
        this.latencies = [];
        
        // Safety
        this.safetyActive = false;
        
        // Key mappings
        this.keyMappings = {
            'w': { axis: 'dz', value: 1 },      // Up
            's': { axis: 'dz', value: -1 },     // Down
            'a': { axis: 'dx', value: -1 },     // Left
            'd': { axis: 'dx', value: 1 },      // Right
            'q': { axis: 'dy', value: 1 },      // Forward
            'e': { axis: 'dy', value: -1 },     // Backward
            'i': { axis: 'dpitch', value: -1 },  // Pitch down
            'k': { axis: 'dpitch', value: 1 },   // Pitch up
            'j': { axis: 'dyaw', value: 1 },     // Yaw left
            'l': { axis: 'dyaw', value: -1 },    // Yaw right
            'u': { axis: 'droll', value: -1 },   // Roll left
            'o': { axis: 'droll', value: 1 },    // Roll right
        };
        
        // VR Input (Supports dual hands)
        this.vrInput = {
            left: { dx: 0, dy: 0, dz: 0, droll: 0, dpitch: 0, dyaw: 0, gripper: -1, reference_frame: null },
            right: { dx: 0, dy: 0, dz: 0, droll: 0, dpitch: 0, dyaw: 0, gripper: -1, reference_frame: null }
        };
        
        // VR Motion Accumulator (for 6-DoF clutch control)
        this.vrAccumulator = {
            left: { dx: 0, dy: 0, dz: 0, droll: 0, dpitch: 0, dyaw: 0 },
            right: { dx: 0, dy: 0, dz: 0, droll: 0, dpitch: 0, dyaw: 0 }
        };
        
        this.init();
    }
    
    init() {
        this.setupUI();
        this.setupKeyboard();
        this.setupButtons();
        this.startStatusUpdates();
    }
    
    setupUI() {
        // Get UI elements
        this.connectionStatus = document.getElementById('connectionStatus');
        this.safetyStatus = document.getElementById('safetyStatus');
        this.commandCountEl = document.getElementById('commandCount');
        this.latencyEl = document.getElementById('latency');
        this.uptimeEl = document.getElementById('uptime');
        this.connectionTimeEl = document.getElementById('connectionTime');
        this.videoPlaceholder = document.getElementById('videoPlaceholder');
        this.videoStream = document.getElementById('videoStream');
        
        // Buttons
        this.connectBtn = document.getElementById('connectBtn');
        this.disconnectBtn = document.getElementById('disconnectBtn');
        this.positionModeBtn = document.getElementById('positionModeBtn');
        this.orientationModeBtn = document.getElementById('orientationModeBtn');
    }
    
    setupKeyboard() {
        // Keyboard events
        document.addEventListener('keydown', (e) => {
            const key = e.key.toLowerCase();
            
            // Special keys
            if (key === 'r') {
                this.resetCommand();
                return;
            }
            if (key === '1') {
                this.activateSafety();
                return;
            }
            if (key === 'm') {
                this.toggleMode();
                return;
            }
            
            // Add to active keys
            if (this.keyMappings[key] && !this.activeKeys.has(key)) {
                this.activeKeys.add(key);
            }
        });
        
        document.addEventListener('keyup', (e) => {
            const key = e.key.toLowerCase();
            this.activeKeys.delete(key);
        });
    }
    
    setupButtons() {
        if (this.connectBtn) {
            this.connectBtn.addEventListener('click', () => {
                this.connect();
            });
        }
        
        if (this.disconnectBtn) {
            this.disconnectBtn.addEventListener('click', () => {
                this.disconnect();
            });
        }
        
        if (this.positionModeBtn) {
            this.positionModeBtn.addEventListener('click', () => {
                this.setMode('position');
            });
        }
        
        if (this.orientationModeBtn) {
            this.orientationModeBtn.addEventListener('click', () => {
                this.setMode('orientation');
            });
        }
        
        // Virtual joystick buttons
        const joystickButtons = document.querySelectorAll('.joystick-btn');
        joystickButtons.forEach(btn => {
            const key = btn.dataset.key;
            if (!key) return;
            
            // Mouse/touch events for virtual buttons
            btn.addEventListener('mousedown', () => this.handleVirtualButton(key, true));
            btn.addEventListener('mouseup', () => this.handleVirtualButton(key, false));
            btn.addEventListener('mouseleave', () => this.handleVirtualButton(key, false));
            
            btn.addEventListener('touchstart', (e) => {
                e.preventDefault();
                this.handleVirtualButton(key, true);
            });
            btn.addEventListener('touchend', (e) => {
                e.preventDefault();
                this.handleVirtualButton(key, false);
            });
        });
    }
    
    handleVirtualButton(key, pressed) {
        if (pressed) {
            // Special actions
            if (key === 'r') {
                this.resetCommand();
                return;
            }
            if (key === '1') {
                this.activateSafety();
                return;
            }
            
            // Add to active keys
            if (this.keyMappings[key]) {
                this.activeKeys.add(key);
            }
        } else {
            this.activeKeys.delete(key);
        }
    }
    
    setToken(token) {
        this.token = token;
    }
    
    setServerUrl(url) {
        try {
            // Ensure URL has protocol
            if (!url.startsWith('http')) {
                const pageProtocol = window.location.protocol; // 'http:' or 'https:'
                url = pageProtocol + '//' + url;
            }
            const urlObj = new URL(url);
            const protocol = urlObj.protocol === 'https:' ? 'wss:' : 'ws:';
            this.wsUrl = `${protocol}//${urlObj.host}/ws/v1/teleop`;
            this.apiUrl = `${urlObj.protocol}//${urlObj.host}/api/v1`;
            console.log(`[TeleopClient] Server URL updated: ${this.wsUrl}`);
        } catch (e) {
            console.error('Invalid URL provided to setServerUrl:', url);
        }
    }

    connect() {
        if (this.connected) return;
        
        let url = this.wsUrl;
        if (this.token) {
            url = `${url}?token=${encodeURIComponent(this.token)}`;
        }
        console.log('Connecting to WebSocket:', url);
        this.updateConnectionStatus('connecting');
        
        try {
            this.ws = new WebSocket(url);
            
            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.connected = true;
                this.reconnectAttempts = 0;
                this.startTime = Date.now();
                this.updateConnectionStatus('connected');
                
                // Start sending commands
                this.startCommandLoop();
                
                // Activate safety on connection
                setTimeout(() => this.activateSafety(), 500);
            };
            
            this.ws.onmessage = (event) => {
                this.handleMessage(JSON.parse(event.data));
            };
            
            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };
            
            this.ws.onclose = () => {
                console.log('WebSocket closed');
                this.connected = false;
                this.updateConnectionStatus('disconnected');
                this.stopCommandLoop();
                
                // Auto-reconnect
                if (this.reconnectAttempts < this.maxReconnectAttempts) {
                    this.reconnectAttempts++;
                    console.log(`Reconnecting... (attempt ${this.reconnectAttempts})`);
                    setTimeout(() => this.connect(), 2000);
                }
            };
        } catch (error) {
            console.error('Failed to connect:', error);
            this.updateConnectionStatus('disconnected');
        }
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.connected = false;
        this.reconnectAttempts = this.maxReconnectAttempts; // Prevent auto-reconnect
        this.stopCommandLoop();
        this.updateConnectionStatus('disconnected');
    }
    
    startCommandLoop() {
        if (this.sendInterval) return;
        
        const interval = 1000 / this.sendFrequency;
        this.sendInterval = setInterval(() => {
            this.sendCommand();
        }, interval);
        
        console.log(`Command loop started at ${this.sendFrequency} Hz`);
    }
    
    stopCommandLoop() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
    }
    
    updateVRInput(input, handedness = "right") {
        const h = handedness.toLowerCase();
        if (this.vrInput[h]) {
            this.vrInput[h] = { ...this.vrInput[h], ...input };
        }
    }

    accumulateVRMotion(delta, handedness = "right") {
        const h = handedness.toLowerCase();
        if (this.vrAccumulator[h]) {
            this.vrAccumulator[h].dx += delta.dx || 0;
            this.vrAccumulator[h].dy += delta.dy || 0;
            this.vrAccumulator[h].dz += delta.dz || 0;
            this.vrAccumulator[h].droll += delta.droll || 0;
            this.vrAccumulator[h].dpitch += delta.dpitch || 0;
            this.vrAccumulator[h].dyaw += delta.dyaw || 0;
        }
    }

    sendCommand() {
        if (!this.connected || !this.ws) return;
        
        // Send commands for both hands
        ["left", "right"].forEach(handedness => {
            const vrIn = this.vrInput[handedness];
            const vrAcc = this.vrAccumulator[handedness];

            // Build command
            const command = {
                dx: 0, dy: 0, dz: 0,
                droll: 0, dpitch: 0, dyaw: 0,
                reference_frame: vrIn.reference_frame || 'end_effector',
                max_velocity: 0.5,
                max_angular_velocity: 1.0,
                timestamp: Date.now() / 1000,
                client_id: 'web_client',
                handedness: handedness
            };
            
            // Only apply keyboard keys to the primary hand (right) for simplicity
            if (handedness === "right") {
                this.activeKeys.forEach(key => {
                    const mapping = this.keyMappings[key];
                    if (!mapping) return;
                    
                    const axis = mapping.axis;
                    const value = mapping.value;
                    
                    const isPosition = ['dx', 'dy', 'dz'].includes(axis);
                    const isOrientation = ['droll', 'dpitch', 'dyaw'].includes(axis);
                    
                    if (this.controlMode === 'position' && isPosition) {
                        command[axis] += value * this.positionIncrement;
                    } else if (this.controlMode === 'orientation' && isOrientation) {
                        command[axis] += value * this.orientationIncrement;
                    }
                });
            }

            // Add VR Input (Rate based)
            command.dx += vrIn.dx;
            command.dy += vrIn.dy;
            command.dz += vrIn.dz;
            command.droll += vrIn.droll;
            command.dpitch += vrIn.dpitch;
            command.dyaw += vrIn.dyaw;
            
            // Add VR Motion (Accumulated delta)
            command.dx += vrAcc.dx;
            command.dy += vrAcc.dy;
            command.dz += vrAcc.dz;
            command.droll += vrAcc.droll;
            command.dpitch += vrAcc.dpitch;
            command.dyaw += vrAcc.dyaw;

            // Reset accumulator for this hand
            this.vrAccumulator[handedness] = {
                dx: 0, dy: 0, dz: 0,
                droll: 0, dpitch: 0, dyaw: 0
            };

            if (vrIn.gripper >= 0) {
                command.gripper_state = vrIn.gripper;
            }
            
            // Only send if there's significant movement or it's the primary hand heartbeat
            const hasMotion = Math.abs(command.dx) > 0.0001 || Math.abs(command.dy) > 0.0001 || 
                              Math.abs(command.dz) > 0.0001 || Math.abs(command.droll) > 0.0001 ||
                              Math.abs(command.dpitch) > 0.0001 || Math.abs(command.dyaw) > 0.0001 ||
                              command.gripper_state !== undefined;

            if (hasMotion || handedness === "right") {
                try {
                    this.lastCommandTime = Date.now();
                    this.ws.send(JSON.stringify(command));
                    this.commandCount++;
                } catch (error) {
                    console.error(`Failed to send ${handedness} command:`, error);
                }
            }
        });

        if (this.commandCountEl) {
            this.commandCountEl.textContent = this.commandCount;
        }
    }
    
    handleMessage(data) {
        // Calculate latency
        if (this.lastCommandTime > 0) {
            const latency = Date.now() - this.lastCommandTime;
            this.latencies.push(latency);
            if (this.latencies.length > 20) {
                this.latencies.shift();
            }
            
            const avgLatency = this.latencies.reduce((a, b) => a + b, 0) / this.latencies.length;
            if (this.latencyEl) {
                this.latencyEl.textContent = `${Math.round(avgLatency)} ms`;
            }
        }
        
        // Update safety status
        if (data.safety_active !== undefined) {
            this.safetyActive = data.safety_active;
            this.updateSafetyStatus(this.safetyActive);
        }
        
        // Handle violations
        if (data.violations) {
            for (const [violation, active] of Object.entries(data.violations)) {
                if (active) {
                    console.warn(`⚠️ ${violation.toUpperCase()}`);
                }
            }
        }
        
        // Handle errors
        if (data.status === 'error') {
            console.error('Server error:', data.message);
        }

        // Handle robot state update (Broadcast)
        if (data.type === 'state' && data.robot) {
            if (this.onStateUpdate) {
                this.onStateUpdate(data.robot);
            }
        }
    }
    
    async activateSafety() {
        try {
            const response = await fetch(`${this.apiUrl}/safety/activate`, {
                method: 'POST',
                headers: {
                    'ngrok-skip-browser-warning': 'true'
                }
            });
            
            if (response.ok) {
                console.log('✓ Safety gate activated');
                this.safetyActive = true;
                this.updateSafetyStatus(true);
            }
        } catch (error) {
            console.error('Failed to activate safety:', error);
        }
    }
    
    async resetCommand() {
        this.activeKeys.clear();
        console.log('Command reset');
    }
    
    toggleMode() {
        this.setMode(this.controlMode === 'position' ? 'orientation' : 'position');
    }
    
    setMode(mode) {
        this.controlMode = mode;
        this.activeKeys.clear();
        
        // Update UI
        const labelEl = document.getElementById('joystickLabel');
        if (mode === 'position') {
            if (this.positionModeBtn) this.positionModeBtn.classList.add('active');
            if (this.orientationModeBtn) this.orientationModeBtn.classList.remove('active');
            if (labelEl) labelEl.textContent = 'Position Control (X, Y, Z)';
        } else {
            if (this.positionModeBtn) this.positionModeBtn.classList.remove('active');
            if (this.orientationModeBtn) this.orientationModeBtn.classList.add('active');
            if (labelEl) labelEl.textContent = 'Orientation Control (Roll, Pitch, Yaw)';
        }
        
        console.log(`Switched to ${mode.toUpperCase()} mode`);
    }
    
    updateConnectionStatus(status) {
        const statusEl = this.connectionStatus;
        
        if (status === 'connected') {
            if (statusEl) {
                statusEl.textContent = 'Connected';
                statusEl.className = 'badge connected';
            }
            if (this.connectBtn) this.connectBtn.style.display = 'none';
            if (this.disconnectBtn) this.disconnectBtn.style.display = 'block';
            if (this.connectionTimeEl) this.connectionTimeEl.textContent = new Date().toLocaleTimeString();
        } else if (status === 'connecting') {
            if (statusEl) {
                statusEl.textContent = 'Connecting...';
                statusEl.className = 'badge disconnected';
            }
        } else {
            if (statusEl) {
                statusEl.textContent = 'Disconnected';
                statusEl.className = 'badge disconnected';
            }
            if (this.connectBtn) this.connectBtn.style.display = 'block';
            if (this.disconnectBtn) this.disconnectBtn.style.display = 'none';
            if (this.connectionTimeEl) this.connectionTimeEl.textContent = '--';
        }
    }
    
    updateSafetyStatus(active) {
        const statusEl = this.safetyStatus;
        
        if (!statusEl) return;
        if (active) {
            statusEl.textContent = 'Safety Active';
            statusEl.className = 'badge safety-active';
        } else {
            statusEl.textContent = 'Safety Inactive';
            statusEl.className = 'badge safety-inactive';
        }
    }
    
    startStatusUpdates() {
        setInterval(() => {
            // Update uptime
            if (this.startTime) {
                const uptime = Math.floor((Date.now() - this.startTime) / 1000);
                const minutes = Math.floor(uptime / 60);
                const seconds = uptime % 60;
                if (this.uptimeEl) {
                    this.uptimeEl.textContent = `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
                }
            }
        }, 1000);
    }
}

// Initialize client when page loads
document.addEventListener('DOMContentLoaded', () => {
    console.log('Initializing Teleoperation Client...');
    window.teleopClient = new TeleopClient();
    
    // Show instructions
    console.log(`
╔════════════════════════════════════════════════════════════╗
║           TELEOPERATION WEB CLIENT                         ║
╠════════════════════════════════════════════════════════════╣
║  CONTROLS:                                                 ║
║  ─────────                                                 ║
║  W/S: Move Up/Down (Z)                                    ║
║  A/D: Move Left/Right (X)                                 ║
║  Q/E: Move Forward/Backward (Y)                           ║
║                                                            ║
║  I/K: Pitch                                               ║
║  J/L: Yaw                                                 ║
║  U/O: Roll                                                ║
║                                                            ║
║  M: Toggle Position/Orientation Mode                       ║
║  R: Reset (Stop all motion)                               ║
║  1: Activate Safety Gate                                  ║
║                                                            ║
║  Or use the virtual joystick buttons on screen!          ║
╚════════════════════════════════════════════════════════════╝
    `);
});
