# Teleop XR ROS 2 Example

This package demonstrates how to integrate `teleop_xr` with ROS 2 and MoveIt 2 for teleoperating a Franka Panda robot.

## Features

- **Fake Hardware**: Runs without a real robot using `mock_components`.
- **MoveIt 2**: Full planning pipeline initialized.
- **Teleop XR**: IK-based teleoperation node running in `ik` mode.
  - **Real-time Feedback**: The headset visualizes the robot's state in real-time, syncing with IK targets when active and actual joint states when inactive.
  - **Camera Streams**: Multiple mock camera streams (head, wrist_left, wrist_right, workspace) are published and displayed in the XR view.
- **RViz**: Visualizes the robot and XR reference frames.

## Changes from Old Example

- **Removed Joint Trajectory Adapter**: The `teleop_xr` node now publishes directly to `/panda_arm_controller/joint_trajectory`.
- **Using Latest `teleop_xr`**: Pointing to the local checkout at `/home/cc/codes/teleop_xr-worktrees/feat-urdf_override`.

## Running the Demo

To launch the full demo stack (Fake Hardware + MoveIt + Teleop XR + RViz):

```bash
pixi run demo
```

This command sources the workspace and runs:
```bash
ros2 launch teleop_xr_ros2_example demo.launch.py
```
