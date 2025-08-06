#!/usr/bin/env python3
"""
steering_controller.py

Listens to /cmd_vel and publishes JointState updates for:
  - front_left_steer_joint
  - front_right_steer_joint

Behavior:
  - While angular.z != 0: steering angle = sign(angular.z) * scale * |angular.z|
    (clamped to max_angle)
  - When angular.z == 0 for more than `hold_timeout`, target ramps back to 0 smoothly
  - Publishes at publish_hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import math

class SteeringController(Node):
    def __init__(self):
        super().__init__('steering_controller')

        # Parameters (tweak as needed)
        self.declare_parameter('max_angle', 0.5)            # radians (≈28.6°)
        self.declare_parameter('angular_scale', 1.0)       # multiplier from cmd_vel.angular.z to angle
        self.declare_parameter('hold_timeout', 0.12)       # seconds: consider still pressing if less than this
        self.declare_parameter('publish_hz', 30.0)         # Hz
        self.declare_parameter('decay_rate', 1.5)          # rad/s - how fast it returns to zero
        self.declare_parameter('right_multiplier', 1.0)    # use 1.0 or -1.0 depending on URDF joint axis

        self.max_angle = self.get_parameter('max_angle').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        self.hold_timeout = self.get_parameter('hold_timeout').get_parameter_value().double_value
        self.publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value
        self.decay_rate = self.get_parameter('decay_rate').get_parameter_value().double_value
        self.right_multiplier = self.get_parameter('right_multiplier').get_parameter_value().double_value

        # Topics
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states_steer', 10)

        # We publish JointState messages (name + position). Using a separate topic avoids colliding with gazebo's joint_states.
        # If your robot_state_publisher expects /joint_states, you can remap or publish there (but that may conflict).
        # Robot state publisher reads joint_states from all publishers; choose whichever works for you.
        # If you prefer /joint_states directly, change topic to '/joint_states'.

        self.last_cmd_time = 0.0
        self.last_angular = 0.0

        # Current (actual) steering angles (we will ramp these smoothly)
        self.cur_left = 0.0
        self.cur_right = 0.0

        # target angles
        self.target = 0.0

        # joint names (must match URDF joint names)
        self.left_joint_name = 'front_left_steer_joint'
        self.right_joint_name = 'front_right_steer_joint'

        period = 1.0 / self.publish_hz
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info('SteeringController started. Publishing steering positions at %.1f Hz' % self.publish_hz)

    def cmd_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_cmd_time = now
        angz = msg.angular.z
        self.last_angular = angz

        # Compute raw target based on angular.z
        raw = angz * self.angular_scale

        # clamp to [-max_angle, max_angle]
        self.target = max(-self.max_angle, min(self.max_angle, raw))

        # when there is angular input, we immediately set the target; actual cur values will ramp to it
        # Note: If you want instant steer set, you can set cur_left/cur_right = target here.
        # We keep ramping for smoothness.

    def timer_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 1.0 / self.publish_hz

        # If no recent cmd, target should be zero (start decay)
        if now - self.last_cmd_time > self.hold_timeout:
            # decay target to zero
            if abs(self.target) > 1e-6:
                # move target toward 0 at rate decay_rate
                sign = math.copysign(1.0, self.target)
                dec = self.decay_rate * dt
                if abs(self.target) <= dec:
                    self.target = 0.0
                else:
                    self.target = self.target - sign * dec

        # Smoothly move current angles toward target
        # limit change per step so steering motion looks natural
        max_step = self.decay_rate * dt  # rad per iteration (same rate)
        # left
        diff_l = self.target - self.cur_left
        if abs(diff_l) <= max_step:
            self.cur_left = self.target
        else:
            self.cur_left += math.copysign(max_step, diff_l)
        # right (may need multiplier sign depending on URDF axis)
        desired_right = self.target * self.right_multiplier
        diff_r = desired_right - self.cur_right
        if abs(diff_r) <= max_step:
            self.cur_right = desired_right
        else:
            self.cur_right += math.copysign(max_step, diff_r)

        # Publish JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint_name, self.right_joint_name]
        js.position = [self.cur_left, self.cur_right]
        # velocities/effort left empty
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
