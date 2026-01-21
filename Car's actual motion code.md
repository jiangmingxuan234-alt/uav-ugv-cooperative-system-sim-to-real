## 🚙 Real-World UGV Implementation (Ackermann Platform)

### Hardware Architecture
This module is designed for an **Ackermann-steering UGV** equipped with a **Raspberry Pi** as the onboard computer. The system utilizes a **2D LiDAR** for real-time environmental perception and obstacle detection.

### Algorithm Evolution
The repository contains several iterations of the control code developed during the testing phase. The file **`小车代码最终版.py` (Final Version)** represents the robust, finalized logic selected for deployment.
* **Core Logic**: Implements a LiDAR-based reactive obstacle avoidance system.
* **Kinematics**: Specifically tuned for Ackermann steering geometry (handling the minimum turning radius and ensuring forward velocity during turns to prevent kinematic locking).
* **Behavior**: Capable of executing autonomous motion loops and effectively avoiding dynamic/static obstacles.

### 🚀 Quick Start Guide (Deployment)
To deploy the code on the physical vehicle, follow these steps:

1.  **Remote Connection**:
    Connect to the onboard Raspberry Pi via **VS Code Remote-SSH** to access the file system.
2.  **Hardware Initialization**:
    * Launch the chassis driver (底层驱动) to enable motor control.
    * Launch the LiDAR driver (e.g., `rplidar_ros` or similar node) to start streaming data.
3.  **Execute Mission**:
    Run the main control script directly:
    ```bash
    python3 小车代码最终版.py
    ```

### Field Test Results
Field testing confirmed stable performance. The UGV successfully achieved reliable obstacle avoidance and completed continuous circular motion trajectories in real-world environments.
