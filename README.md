# Robot Autonomy Project Spring 2025

This project was part of the Robot Autonomy course at Carnegie Mellon University. The goal of the project was to precisely pour an arbitrary amount of beads into a container,
to serve as a foundation for other, more complex, pouring tasks. To accomplish this, we modeled the bead pouring dynamics, and used a direct collocation technique to optimize
a pitch trajectory for the robot manipulator to follow. A PD controller was then used to track the desired trajectory.

### Project Report:
[Report Link](https://drive.google.com/file/d/1pgp-c3Z5qVecwIIcbcmdoBHYkJPaO3JG/view?usp=sharing)

### Demonstration Video:
[Video](https://drive.google.com/file/d/1AO35xT9SAFWQ5qzlmW-o09ssS_QAiykP/view?usp=sharing)

### To Execute Code:
- Follow instructions on starting FrankaPy and Franka-Interface
- In docker, in the teamC25-robot-autonomy folder
  - "source commands.sh"
  - Enter Y, and 23 to prompts
- Launch Realsense, MoveIt, and TF publisher nodes
- run "python3 main.py" and follow prompts in the command line.
