"""
Simple 3D Visualizer for Teleoperation System
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import requests
import numpy as np
import time
import threading

# Configuration
SERVER_URL = "http://localhost:8000"
UPDATE_INTERVAL = 100  # ms

# Workspace Limits (should match server/models.py)
WS_MIN_X, WS_MAX_X = -1.0, 1.0
WS_MIN_Y, WS_MAX_Y = -1.0, 1.0
WS_MIN_Z, WS_MAX_Z = 0.0, 1.5

class TeleopVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Data
        self.position = np.array([0.0, 0.0, 0.5])
        self.trajectory = []
        self.max_trajectory_length = 100
        self.connected = False
        
        # Setup plot
        self.setup_plot()
        
        # Start data polling thread
        self.running = True
        self.poll_thread = threading.Thread(target=self.poll_data, daemon=True)
        self.poll_thread.start()
        
        # Start animation
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=UPDATE_INTERVAL)
        
    def setup_plot(self):
        """Initialize the plot"""
        self.ax.set_title("Teleoperation Visualizer (Mock)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        
        # Set limits slightly larger than workspace
        self.ax.set_xlim(WS_MIN_X - 0.2, WS_MAX_X + 0.2)
        self.ax.set_ylim(WS_MIN_Y - 0.2, WS_MAX_Y + 0.2)
        self.ax.set_zlim(WS_MIN_Z - 0.2, WS_MAX_Z + 0.2)
        
        # Draw workspace box
        self.draw_workspace_box()
        
        # Robot end-effector marker
        self.scat = self.ax.scatter([0], [0], [0.5], c='r', marker='o', s=100, label='End Effector')
        
        # Trajectory line
        self.traj_line, = self.ax.plot([], [], [], 'b-', alpha=0.5, label='Trajectory')
        
        self.ax.legend()
        
    def draw_workspace_box(self):
        """Draw the workspace boundaries"""
        # Corners of the box
        corners = [
            [WS_MIN_X, WS_MIN_Y, WS_MIN_Z],
            [WS_MAX_X, WS_MIN_Y, WS_MIN_Z],
            [WS_MAX_X, WS_MAX_Y, WS_MIN_Z],
            [WS_MIN_X, WS_MAX_Y, WS_MIN_Z],
            [WS_MIN_X, WS_MIN_Y, WS_MAX_Z],
            [WS_MAX_X, WS_MIN_Y, WS_MAX_Z],
            [WS_MAX_X, WS_MAX_Y, WS_MAX_Z],
            [WS_MIN_X, WS_MAX_Y, WS_MAX_Z]
        ]
        
        # Edges connecting corners
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0], # Bottom face
            [4, 5], [5, 6], [6, 7], [7, 4], # Top face
            [0, 4], [1, 5], [2, 6], [3, 7]  # Vertical edges
        ]
        
        for edge in edges:
            x = [corners[edge[0]][0], corners[edge[1]][0]]
            y = [corners[edge[0]][1], corners[edge[1]][1]]
            z = [corners[edge[0]][2], corners[edge[1]][2]]
            self.ax.plot(x, y, z, 'g--', alpha=0.3)
            
    def poll_data(self):
        """Poll server for robot status"""
        while self.running:
            try:
                response = requests.get(f"{SERVER_URL}/api/v1/statistics", timeout=0.5)
                if response.status_code == 200:
                    data = response.json()
                    # Check where the position is stored based on server response structure
                    # server/teleop_server.py: get_statistics returns:
                    # 'backend_status': self.backend.get_status() -> 'current_position'
                    
                    backend_status = data.get('backend_status', {})
                    pos = backend_status.get('current_position')
                    
                    if pos:
                        self.position = np.array(pos)
                        self.connected = True
                        
                        # Update trajectory
                        self.trajectory.append(self.position)
                        if len(self.trajectory) > self.max_trajectory_length:
                            self.trajectory.pop(0)
                    else:
                        # Fallback to controller stats if backend doesn't have it
                        controller_stats = data.get('controller_stats', {})
                        pos = controller_stats.get('current_position')
                        if pos:
                            self.position = np.array(pos)
                            self.connected = True
                            
                            self.trajectory.append(self.position)
                            if len(self.trajectory) > self.max_trajectory_length:
                                self.trajectory.pop(0)
                else:
                    self.connected = False
            except Exception as e:
                self.connected = False
                # print(f"Connection error: {e}")
            
            time.sleep(0.1)

    def update_plot(self, frame):
        """Update the plot with new data"""
        # Update title with connection status and position
        status = "Connected" if self.connected else "Disconnected"
        self.ax.set_title(f"Teleoperation Visualizer (Mock) - {status}\n"
                          f"Pos: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]")
        
        # Update scatter position using the internal _offsets3d property for 3D scatter
        self.scat._offsets3d = ([self.position[0]], [self.position[1]], [self.position[2]])
        
        # Update trajectory
        if len(self.trajectory) > 1:
            traj_arr = np.array(self.trajectory)
            self.traj_line.set_data(traj_arr[:, 0], traj_arr[:, 1])
            self.traj_line.set_3d_properties(traj_arr[:, 2])
            
        return self.scat, self.traj_line

    def run(self):
        """Run the visualizer"""
        print("Starting visualizer...")
        print(f"Connecting to {SERVER_URL}...")
        plt.show()
        self.running = False
        self.poll_thread.join()

if __name__ == "__main__":
    viz = TeleopVisualizer()
    viz.run()
