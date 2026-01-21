# UAV–UGV Cooperative System (Sim-to-Real)

## Overview
This repository presents a **UAV–UGV cooperative system with sim-to-real validation**.
The system is developed and verified through both **RflySim-based simulation** and
**real-world experiments**, focusing on air–ground coordination, perception-driven
decision making, and robust motion control.

The project aims to bridge the gap between simulation and real deployment for
multi-robot cooperative systems.

---

## System Architecture
The system consists of the following main modules:

- **Decision Layer**
  - State-machine-based task coordination
  - Mission logic for UAV–UGV cooperation

- **Perception Layer**
  - LiDAR-based obstacle detection
  - Vision-based target/frame recognition (optional)

- **Control Layer**
  - UAV position/velocity control via PX4 & MAVROS
  - UGV motion control and trajectory tracking

- **Simulation & Real Deployment**
  - RflySim for rapid prototyping and validation
  - Real-world experiments for sim-to-real verification

> A high-level system architecture diagram will be provided in `docs/`.

---

## Sim-to-Real Validation
This project emphasizes **sim-to-real transfer**:

- Algorithms are first validated in simulation
- The same control logic is deployed to real UAV–UGV platforms
- System robustness is evaluated under real-world uncertainty

Representative experiment results (photos / videos) will be added for demonstration.

---

## Project Structure
```text
uav-ugv-cooperative-system-sim-to-real/
├─ docs/               # Architecture diagrams and experiment media
├─ simulation/         # RflySim configuration and simulation scripts
├─ control/            # UAV & UGV control logic
├─ perception/         # Perception and obstacle avoidance modules
├─ requirements.txt
└─ README.md
