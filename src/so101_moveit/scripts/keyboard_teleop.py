#!/usr/bin/env python3
"""
Improved keyboard teleoperation for SO-101 robot.
Multi-key support with smooth, responsive control.
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from pynput import keyboard
from threading import Lock
from queue import Queue

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Joint names
        self.arm_joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll'
        ]
        self.gripper_joint_name = 'gripper'

        # Joint limits (from URDF)
        self.joint_limits = {
            'shoulder_pan': (-1.91986, 1.91986),    # ±110°
            'shoulder_lift': (-1.74533, 1.74533),   # ±100°
            'elbow_flex': (-1.74533, 1.5708),       # -100° to 90°
            'wrist_flex': (-1.65806, 1.65806),      # ±95°
            'wrist_roll': (-2.79253, 2.79253),      # ±160°
            'gripper': (-0.1745, 1.4483)            # -10° to 83°
        }

        # Current joint positions (read from robot)
        self.current_positions = {name: 0.0 for name in self.arm_joint_names}
        self.current_positions[self.gripper_joint_name] = 0.0
        self.position_lock = Lock()

        # Target joint positions (what we're commanding) - ACCUMULATOR PATTERN
        self.target_positions = {name: 0.0 for name in self.arm_joint_names}
        self.target_positions[self.gripper_joint_name] = 0.0
        self.target_lock = Lock()
        self.targets_initialized = False

        # ROS subscribers and publishers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.arm_traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Gripper uses GripperActionController - use action client
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # Movement increments (fine control for precision)
        self.arm_increment = 0.04  # ~2.3 degrees per update at 30 Hz
        self.gripper_increment = 0.10  # Gripper uses separate action controller

        # Event queue for keyboard events
        self.event_queue = Queue()
        self.pressed_keys = {}  # key -> is_pressed
        self.keys_lock = Lock()

        # Keyboard listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

        # Update timer (20 Hz - slower to reduce jitter)
        self.timer = self.create_timer(1.0/20.0, self.update_control)

        # Initialize targets after a delay
        self.init_timer = self.create_timer(0.5, self.initialize_targets)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Keyboard Teleoperation Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Controls:')
        self.get_logger().info('  Q/A - shoulder_pan     (left/right)')
        self.get_logger().info('  W/S - shoulder_lift    (down/up)')
        self.get_logger().info('  E/D - elbow_flex       (extend/retract)')
        self.get_logger().info('  R/F - wrist_flex       (down/up)')
        self.get_logger().info('  T/G - wrist_roll       (rotate CCW/CW)')
        self.get_logger().info('  O/L - Gripper          (CLOSE/OPEN)')
        self.get_logger().info('  ESC - Exit')
        self.get_logger().info('=' * 60)
        self.get_logger().info('TIP: Hold multiple keys at once for combined motion!')
        self.get_logger().info('=' * 60)

    def clamp_joint_position(self, joint_name, position):
        """Clamp joint position to its limits."""
        if joint_name in self.joint_limits:
            lower, upper = self.joint_limits[joint_name]
            return max(lower, min(upper, position))
        return position

    def initialize_targets(self):
        """Initialize target positions from current positions."""
        with self.position_lock:
            with self.target_lock:
                self.target_positions.update(self.current_positions)
                self.targets_initialized = True
        self.get_logger().info('Ready for control!')
        # Cancel the initialization timer after running once
        self.init_timer.cancel()

    def joint_state_callback(self, msg):
        """Update current joint positions."""
        with self.position_lock:
            for i, name in enumerate(msg.name):
                if name in self.current_positions:
                    self.current_positions[name] = msg.position[i]

    def on_press(self, key):
        """Handle key press - add to event queue."""
        try:
            if hasattr(key, 'char') and key.char:
                self.event_queue.put((key.char.lower(), True))
            elif key == keyboard.Key.esc:
                self.get_logger().info('ESC pressed - shutting down...')
                self.listener.stop()
                rclpy.shutdown()
                sys.exit(0)
        except Exception:
            pass

    def on_release(self, key):
        """Handle key release - add to event queue."""
        try:
            if hasattr(key, 'char') and key.char:
                self.event_queue.put((key.char.lower(), False))
        except Exception:
            pass

    def drain_events(self):
        """Process all events in the queue."""
        while not self.event_queue.empty():
            key, is_pressed = self.event_queue.get()
            with self.keys_lock:
                if is_pressed:
                    self.pressed_keys[key] = True
                else:
                    self.pressed_keys.pop(key, None)

    def update_control(self):
        """Update robot control based on pressed keys."""
        if not self.targets_initialized:
            return

        # Process all keyboard events
        self.drain_events()

        # Get currently pressed keys
        with self.keys_lock:
            pressed = list(self.pressed_keys.keys())

        if not pressed:
            return

        # Key mappings
        arm_key_map = {
            'q': ('shoulder_pan', -1),
            'a': ('shoulder_pan', 1),
            'w': ('shoulder_lift', -1),
            's': ('shoulder_lift', 1),
            'e': ('elbow_flex', -1),
            'd': ('elbow_flex', 1),
            'r': ('wrist_flex', -1),
            'f': ('wrist_flex', 1),
            't': ('wrist_roll', -1),
            'g': ('wrist_roll', 1),
        }

        gripper_key_map = {
            'o': -1,  # close
            'l': 1,   # open
        }

        # Update target positions (ACCUMULATOR PATTERN)
        with self.target_lock:
            arm_changed = False
            gripper_changed = False

            for key in pressed:
                if key in arm_key_map:
                    joint_name, direction = arm_key_map[key]
                    self.target_positions[joint_name] += direction * self.arm_increment
                    # Clamp to joint limits
                    self.target_positions[joint_name] = self.clamp_joint_position(
                        joint_name, self.target_positions[joint_name]
                    )
                    arm_changed = True
                elif key in gripper_key_map:
                    direction = gripper_key_map[key]
                    self.target_positions[self.gripper_joint_name] += direction * self.gripper_increment
                    # Clamp to joint limits
                    self.target_positions[self.gripper_joint_name] = self.clamp_joint_position(
                        self.gripper_joint_name, self.target_positions[self.gripper_joint_name]
                    )
                    gripper_changed = True

            # Publish arm trajectory
            if arm_changed:
                traj = JointTrajectory()
                traj.joint_names = self.arm_joint_names

                point = JointTrajectoryPoint()
                point.positions = [self.target_positions[name] for name in self.arm_joint_names]
                point.time_from_start.sec = 0
                point.time_from_start.nanosec = 200_000_000  # 0.2 seconds - smoother motion

                traj.points.append(point)
                self.arm_traj_pub.publish(traj)

                # Display commanded positions
                print("\r\033[K", end='')  # Clear line
                pos_str = " | ".join([
                    f"{name[0:3].upper()}: {self.target_positions[name]:+6.3f}"
                    for name in self.arm_joint_names
                ])
                print(f"ARM: {pos_str}", end='', flush=True)

            # Send gripper command via action (separate controller)
            if gripper_changed:
                goal_msg = GripperCommand.Goal()
                goal_msg.command.position = self.target_positions[self.gripper_joint_name]
                goal_msg.command.max_effort = 50.0  # Reasonable effort limit

                # Send goal asynchronously (non-blocking)
                self.gripper_action_client.send_goal_async(goal_msg)

                # Display gripper position
                print("\r\033[K", end='')  # Clear line
                print(f"GRIPPER: {self.target_positions[self.gripper_joint_name]:+6.3f}", end='', flush=True)

def main(args=None):
    # Disable terminal echo to prevent key characters from appearing
    old_settings = None
    if sys.stdin.isatty():
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass

    rclpy.init(args=args)

    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        if old_settings and sys.stdin.isatty():
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
