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

## TODO

### Servo Motion Smoothing (Feetech STS3215)

The SO-101 uses Feetech STS3215 servos which produce jerky motion compared to industrial robot arms with harmonic drives or QDDs. Research shows several improvement strategies:

#### Quick Wins (Parameter Tuning)
- [ ] **Lower ACC value**: Change from `50` to `10-20` in `so101_system.cpp` for smoother acceleration/deceleration ramps
- [ ] **Reduce Speed**: Change from `2400` to `800-1000` for slower but smoother motion
- [ ] **Make ACC/Speed configurable**: Expose as ROS parameters instead of hardcoded values

#### Advanced Improvements
- [ ] **Use GOAL_TIME mode**: Send time duration instead of speed to the servos - ensures all joints arrive simultaneously regardless of travel distance
- [ ] **Host-side trajectory interpolation**: Implement S-curve or cubic spline interpolation on the host to generate smooth intermediate waypoints at high frequency (200-500Hz)

#### SDK Capabilities (Already Available)
The Feetech SCServo SDK supports:
| Parameter | Register | Function |
|-----------|----------|----------|
| ACC | 0x29 (41) | Acceleration value (0-254) for soft start/stop |
| GOAL_POSITION | 0x2A-0x2B | Target position |
| GOAL_TIME | 0x2C-0x2D | Target time to reach position (alternative to speed) |
| GOAL_SPEED | 0x2E-0x2F | Maximum speed for movement |

#### Hardware Limitations (Cannot Fix with Software)
- Plastic gears → backlash causes micro-jitter at direction reversals
- 12-bit encoder (4096 steps) → ~0.088° resolution, visible on fine movements
- Trapezoidal profile only → no true S-curve in servo firmware
- ~1kHz internal control loop → vs 10kHz+ on industrial arms

#### Notes
- Very low ACC values (e.g., ACC=2) can cause **overheating** during continuous motion
- Recommended ACC range: **10-30** for balance of smoothness and thermal performance
- Best possible improvement: host-side interpolation (~50-70% smoother)

## License
This project follows the LeRobot license (Apache 2.0).

## Acknowledgments
- **LeRobot**: [GitHub](https://github.com/huggingface/lerobot)
- **LeRobot-ROS**: [PR #8](https://github.com/ycheng517/lerobot-ros)
- **MoveIt 2**: [Documentation](https://moveit.picknik.ai/main/index.html)
- **LycheeAI Hub**: [SO-ARM101: MoveIt in Isaac Sim with ROS2](https://lycheeai-hub.com/isaac-sim/ros2/so-arm101-moveit-in-isaac-sim-with-ros2)
- **SO-ARM101_MoveIt_IsaacSim**: [GitHub](https://github.com/MuammerBay/SO-ARM101_MoveIt_IsaacSim)
- **SCServo_Linux**: [GitHub](https://github.com/adityakamath/SCServo_Linux)