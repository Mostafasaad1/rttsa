# RT Trajectory Smoothing Algorithim

A comprehensive toolchain for high-performance robotic motion planning, featuring a deterministic C++ S-curve library, a modern PySide6 HMI, and real-time visualization in CoppeliaSim.

## Demo

https://github.com/user-attachments/assets/dd92af0d-00bc-49b3-8132-ee313ab39648

*Real-time trajectory smoothing with S-curve motion profiles visualized in CoppeliaSim*

## Project Components

### 1. Core S-Curve Library (C++)
A high-performance library for real-time trajectory planning with 7-segment S-curve motion profiles.
- **O(1) Deterministic Sampling**: State evaluation in constant time, suitable for 1kHz+ control loops.
- **Multi-Axis Synchronization**: Coordinated motion across multiple degrees of freedom.
- **Mid-Motion Replanning**: Seamlessly update targets from any non-zero state.

### 2. Multi-Waypoint Bezier Planner
Extends Cartesian planning to support complex paths through up to 10 user-defined waypoints.
- **Catmull-Rom Interpolation**: Ensures C1 continuity (smooth velocity) through all points.
- **Piecewise Cubic Splines**: Generates optimal Cartesian paths with customizable arc heights.
- **Real-Time IK Solver**: Damped Least Squares IK for the UR5 robot arm.

### 3. Robotic HMI (PySide6)
A premium dark-themed dashboard for real-time monitoring and control.
- **Interactive Previews**: 4-panel projection (Main 3D + Top/Front/Side views).
- **Adaptive UI**: Responsive layouts using `QSplitter` and adaptive widgets.
- **Live Simulation Bridge**: Streams joint-space states via UDP to CoppeliaSim.

## Installation

### Prerequisites
- **C++**: CMake 3.14+, C++17 compiler.
- **Python**: PySide6, NumPy, Matplotlib.
- **Robotics**: CoppeliaSim (for simulation).
- **Libraries**: Eigen3, Robotics Library (RL) 0.7.0.

### Building the C++ Backend
**Important:** You must build the project before running the GUI.

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

This compiles the S-curve library and test runners needed by the GUI.

### Python Dependencies
```bash
pip install PySide6 numpy matplotlib
# For CoppeliaSim bridge integration:
pip install coppeliasim-zmqremoteapi-client
```

## Quick Start

**Note:** Make sure you've built the project first (see Building section above).

### Option 1: Integrated Bridge (Recommended)
1. **Launch CoppeliaSim**: Open the provided scene in `coppeliasim scene/`.
2. **Launch the HMI**:
   ```bash
   python3 gui.py
   ```
3. **Enable Bridge**: In the GUI, check "Enable Integrated Bridge" in the CoppeliaSim Bridge section.
4. **Plan & Execute**:
   - Use the **Joint-Space** tab for simple point-to-point motion.
   - Use the **Cartesian Spline Planner** tab to define a complex 3D path with multiple waypoints and execute it on the simulated UR5.

### Option 2: Standalone Bridge
1. **Launch CoppeliaSim**: Open the provided scene in `coppeliasim scene/`.
2. **Start the Sim Bridge** (in a separate terminal):
   ```bash
   python3 rt_sim_runner.py
   ```
3. **Launch the HMI**:
   ```bash
   python3 gui.py
   ```
4. **Plan & Execute**: Use the GUI tabs as described above.

## Project Structure
- `src/`: C++ S-Curve library source.
- `tests/`: Performance tests, IK solvers, and path runners.
- `gui.py`: Main PySide6 HMI application.
- `rt_sim_runner.py`: ZeroMQ/UDP bridge to CoppeliaSim.
- `models/`: Robot kinematics and simulation models.

## Requirements
- C++17 compatible compiler
- CMake 3.14 or later
- GoogleTest (automatically fetched)

## License
Distributed under the MIT License. See `LICENSE` for more information.
