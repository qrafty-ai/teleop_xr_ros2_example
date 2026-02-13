# Teleop XR ROS 2 Example

This repository demonstrates a modern ROS 2 Humble workspace integrated with `teleop_xr` for intuitive robot teleoperation using WebXR.

![Teleop XR Demo](./src/teleop_xr_ros2_example/demo.gif)

## Features

- **Pixi & UV**: Seamless environment management for ROS 2 and Python.
- **IK-based Teleoperation**: Uses `teleop_xr` with a high-performance IK solver for Franka Panda.
- **Real-time Visualization**: Synced robot state visualization in both RViz and the XR headset.
- **Mock Hardware**: Includes a mock camera publisher and robot components for running without a physical robot.

## Prerequisites

- Install [Pixi](https://pixi.sh/)
- Install [UV](https://docs.astral.sh/uv/)

## Getting Started

### 1. Setup the Environment
Initialize the Pixi environment and Python virtual environment:
```bash
pixi run setup
```

### 2. Build the Workspace
Build the ROS 2 packages:
```bash
pixi run build
```

### 3. Run the Demo
Launch the full simulation stack (MoveIt 2 + Teleop XR + RViz):
```bash
pixi run demo
```

Once running:
1. Open the URL printed by the `teleop_xr` node (default: `https://<your-ip>:4443`) in your VR/AR headset.
2. Enter VR mode.
3. Use the controllers to command the robot.

## Project Structure

- `src/teleop_xr_ros2_example/`: Main ROS 2 package.
  - `launch/demo.launch.py`: Main entry point.
  - `config/`: MoveIt and controller configurations.
- `pixi.toml`: Workspace configuration and task definitions.
- `scripts/setup.sh`: Environment initialization script.

## Acknowledgments

- [teleop_xr](https://github.com/qrafty-ai/teleop_xr): Core teleoperation library.
- [pyroki](https://github.com/qrafty-ai/pyroki): Differentiable IK solver.
