Demo Usage & Workflow Description
🚀 Quick Start Guide
Environment Setup: Ensure the toolchain provided in the manual is correctly installed.

Launch Simulation: Run the SITL (Software In The Loop) script.

Indicator: Wait until QGroundControl (QGC) displays "Ready to fly" in the top-left corner. This confirms the connection is established and the vehicle is healthy.

Execute Mission: Run the offboard script.

The system will sequentially execute the mission stages: Offboard Control -> Obstacle Avoidance (bz) -> Balloon Detection.

📂 Key Scripts & Modules
1. Core Mission Scripts
offboard.py (Gate Crossing with LiDAR)

Function: Handles takeoff, initial hovering, and cruising.

Logic: Uses LiDAR point clouds (visualized in RViz) to detect the gate location and executes a "fly-through" maneuver automatically.

bz.py (Obstacle Avoidance / Navigation)

Function: Designed for navigating through the obstacle field.

Note: While originally intended to implement dynamic obstacle avoidance, the current version uses a pre-defined fixed trajectory to ensure stability and mission completion after multiple experimental iterations.

balloon_detect.py (Target Engagement)

Function: Identifies red balloons using the downward-facing camera.

Action: Upon detection, it performs the precision maneuvering required to pop the balloon, marking the end of the simulation mission.

2. Environment & Utility Scripts
Envset.py (Scene Initialization)

Function: Initializes the simulation environment via the API.

Details: Spawns static obstacles (pillars) and generates a dynamic car that moves along a preset cyclic trajectory to simulate a dynamic environment.

shutdown

Function: A utility script to strictly close the simulation platform and kill all related processes with one click.

3. Legacy / Experimental Code
sj.py (Visual Gate Detection)

Status: Deprecated / Unused.

Details: An experimental script attempting to use Canny Edge Detection + Contour Analysis for gate crossing. This approach was ultimately replaced by the LiDAR-based solution (offboard.py) for better reliability, but the code is kept for reference.

⚙️ Configuration
Config Files: The remaining files in this folder are parameter configuration files. Please refer to the user manual for detailed explanations of each parameter.
