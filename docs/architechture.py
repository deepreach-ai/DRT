'''
┌─────────────────────────────────────────────────────┐
│                    CLIENT                           │
│  (Keyboard/Gamepad/Script sending delta commands)  │
└─────────────────┬───────────────────────────────────┘
                  │ HTTP/WebSocket
                  ↓
┌─────────────────────────────────────────────────────┐
│           TELEOPERATION SERVER (AWS)                │
│                                                     │
│  ┌──────────────────────────────────────────────┐  │
│  │  Input Handler                               │  │
│  │  - Receive delta commands                    │  │
│  │  - Deadman switch validation                 │  │
│  └──────────────┬───────────────────────────────┘  │
│                 ↓                                   │
│  ┌──────────────────────────────────────────────┐  │
│  │  Control Logic                               │  │
│  │  - Apply workspace limits                    │  │
│  │  - Apply velocity limits                     │  │
│  │  - Compute target pose                       │  │
│  └──────────────┬───────────────────────────────┘  │
│                 ↓                                   │
│  ┌──────────────────────────────────────────────┐  │
│  │  Robot Backend Interface (Abstract)          │  │
│  │  - execute_command(target_pose)              │  │
│  │  - get_current_state()                       │  │
│  └──────────────┬───────────────────────────────┘  │
└─────────────────┼───────────────────────────────────┘
                  │
         ┌────────┴────────┐
         ↓                 ↓
┌──────────────────┐  ┌──────────────────┐
│ IsaacSimBackend  │  │ MockRobotBackend │
│ - Isaac Sim API  │  │ - Simulated      │
│ - IK solver      │  │ - Logging only   │
└──────────────────┘  └──────────────────┘
'''