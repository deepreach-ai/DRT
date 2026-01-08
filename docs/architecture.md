```
[Your Local Machine]          [AWS EC2 g5.4xlarge]
     |                              |
Keyboard Client  ------>  Teleop Server (port 8000)
                                    |
                          Isaac Backend (port 9000) <----- Isaac Sim Client
                                    |                           |
                                    |                      [Simulated Robot]
                                    └-- Command Integration