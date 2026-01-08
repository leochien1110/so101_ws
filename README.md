# SO-101 Robot Control Workspace (so101_ws)

This workspace contains the ROS 2 packages for controlling the SO-101 robot arm using Feetech servos, MoveIt 2, and `ros2_control`.


## Packages
- **`so101_hardware`**: The C++ hardware interface driver for Feetech servos. Supports LeRobot JSON calibration.
- **`so101_description`**: URDF description and meshes for the SO-101 robot.
- **`so101_moveit`**: MoveIt 2 configuration and launch files.

## Getting Started

### 1. Build
```bash
cd so101_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch
To launch the real robot control stack:
```bash
ros2 launch so101_moveit demo.launch.py \
  use_fake_hardware:=false \
  port:=/dev/ttyACM1
```

*(Note: Ensure your user is in the `dialout` group to access the serial port.)*

## Key Configurations

### Inverse Kinematics (MoveIt)
We use the **LMA (Levenberg-Marquardt)** solver for robust 5-DOF control.
- **Solver**: `lma_kinematics_plugin/LMAKinematicsPlugin`
- **Config**: `src/so101_moveit/config/kinematics.yaml`
- **Critical Fix**: A dummy `world` link was added to the URDF to prevent solver errors related to root linkage inertia.
- **Approximate IK Solutions**: Since the SO-101 is a 5-DOF arm, we use the `approximate_ik_solutions` parameter to user to drag the marker to the desired position without too many restrictions. To be noted, this may result in the robot rotating in an unexpected way, i.e. the robot may rotate the gripper even you just drag the translation arrows.

### Calibration
The driver natively reads LeRobot JSON calibration files.
- **Default File**: `src/so101_moveit/config/calibration_default.json`. You should replace it with your lerobot follower calibration file (e.g. `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/my_awesome_follower_arm.json`).
- **Logic**: Maps raw motor values (0-4095) to URDF radian limits using `range_min` and `range_max`.

## License
This project follows the LeRobot license (Apache 2.0).

## Acknowledgments
- **LeRobot**: [GitHub](https://github.com/huggingface/lerobot)
- **LeRobot-ROS**: [PR #8](https://github.com/ycheng517/lerobot-ros)
- **MoveIt 2**: [Documentation](https://moveit.picknik.ai/main/index.html)
- **LycheeAI Hub**: [SO-ARM101: MoveIt in Isaac Sim with ROS2](https://lycheeai-hub.com/isaac-sim/ros2/so-arm101-moveit-in-isaac-sim-with-ros2)
- **SO-ARM101_MoveIt_IsaacSim**: [GitHub](https://github.com/MuammerBay/SO-ARM101_MoveIt_IsaacSim)
- **SCServo_Linux**: [GitHub](https://github.com/adityakamath/SCServo_Linux)