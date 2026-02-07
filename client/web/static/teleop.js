// Teleoperation Web Client
class TeleopClient {
    constructor() {
        // Configuration - get server URL from page or default to localhost
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.hostname || 'localhost';
        const port = window.location.port || '8000';
        this.wsUrl = `${protocol}//${host}:${port}/ws/v1/teleop`;
        this.apiUrl = `${window.location.protocol}//${host}:${port}/api/v1`;
        
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
        this.sendFrequency = 20; // Hz
        
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
        
        // VR Input
        this.vrInput = {
            dx: 0, dy: 0, dz: 0,
            droll: 0, dpitch: 0, dyaw: 0,
            gripper: -1
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
        // Connect button
        this.connectBtn.addEventListener('click', () => {
            this.connect();
        });
        
        // Disconnect button
        this.disconnectBtn.addEventListener('click', () => {
            this.disconnect();
        });
        
        // Mode buttons
        this.positionModeBtn.addEventListener('click', () => {
            this.setMode('position');
        });
        
        this.orientationModeBtn.addEventListener('click', () => {
            this.setMode('orientation');
        });
        
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
    
    connect() {
        if (this.connected) return;
        
        console.log('Connecting to WebSocket:', this.wsUrl);
        this.updateConnectionStatus('connecting');
        
        try {
            this.ws = new WebSocket(this.wsUrl);
            
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
    
    updateVRInput(input) {
        this.vrInput = { ...this.vrInput, ...input };
    }

    sendCommand() {
        if (!this.connected || !this.ws) return;
        
        // Build command from active keys
        const command = {
            dx: 0, dy: 0, dz: 0,
            droll: 0, dpitch: 0, dyaw: 0,
            reference_frame: 'end_effector',
            max_velocity: 0.5,
            max_angular_velocity: 1.0,
            timestamp: Date.now() / 1000,
            client_id: 'web_client'
        };
        
        // Apply active keys based on control mode
        this.activeKeys.forEach(key => {
            const mapping = this.keyMappings[key];
            if (!mapping) return;
            
            const axis = mapping.axis;
            const value = mapping.value;
            
            // Check if this axis is for position or orientation
            const isPosition = ['dx', 'dy', 'dz'].includes(axis);
            const isOrientation = ['droll', 'dpitch', 'dyaw'].includes(axis);
            
            if (this.controlMode === 'position' && isPosition) {
                command[axis] += value * this.positionIncrement;
            } else if (this.controlMode === 'orientation' && isOrientation) {
                command[axis] += value * this.orientationIncrement;
            }
        });

        // Add VR Input
        command.dx += this.vrInput.dx;
        command.dy += this.vrInput.dy;
        command.dz += this.vrInput.dz;
        command.droll += this.vrInput.droll;
        command.dpitch += this.vrInput.dpitch;
        command.dyaw += this.vrInput.dyaw;
        if (this.vrInput.gripper >= 0) {
            command.gripper_state = this.vrInput.gripper;
        }
        
        // Send command (even if zero - acts as heartbeat)
        try {
            this.lastCommandTime = Date.now();
            this.ws.send(JSON.stringify(command));
            this.commandCount++;
            this.commandCountEl.textContent = this.commandCount;
        } catch (error) {
            console.error('Failed to send command:', error);
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
            this.latencyEl.textContent = `${Math.round(avgLatency)} ms`;
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
    }
    
    async activateSafety() {
        try {
            const response = await fetch(`${this.apiUrl}/safety/activate`, {
                method: 'POST'
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
        if (mode === 'position') {
            this.positionModeBtn.classList.add('active');
            this.orientationModeBtn.classList.remove('active');
            document.getElementById('joystickLabel').textContent = 'Position Control (X, Y, Z)';
        } else {
            this.positionModeBtn.classList.remove('active');
            this.orientationModeBtn.classList.add('active');
            document.getElementById('joystickLabel').textContent = 'Orientation Control (Roll, Pitch, Yaw)';
        }
        
        console.log(`Switched to ${mode.toUpperCase()} mode`);
    }
    
    updateConnectionStatus(status) {
        const statusEl = this.connectionStatus;
        
        if (status === 'connected') {
            statusEl.textContent = 'Connected';
            statusEl.className = 'badge connected';
            this.connectBtn.style.display = 'none';
            this.disconnectBtn.style.display = 'block';
            this.connectionTimeEl.textContent = new Date().toLocaleTimeString();
        } else if (status === 'connecting') {
            statusEl.textContent = 'Connecting...';
            statusEl.className = 'badge disconnected';
        } else {
            statusEl.textContent = 'Disconnected';
            statusEl.className = 'badge disconnected';
            this.connectBtn.style.display = 'block';
            this.disconnectBtn.style.display = 'none';
            this.connectionTimeEl.textContent = '--';
        }
    }
    
    updateSafetyStatus(active) {
        const statusEl = this.safetyStatus;
        
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
                this.uptimeEl.textContent = `${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
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
