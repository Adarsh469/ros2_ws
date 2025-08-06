#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select, time
import math

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self._timer_cb)  # 20Hz

        # targets and actuals
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.lin = 0.0
        self.ang = 0.0

        # params (tweak these)
        self.max_lin = 0.8            # m/s
        self.max_ang = 2.0            # rad/s (increased for stronger turning)
        self.linear_accel = 0.06      # ramp per cycle
        self.linear_decel = 0.12
        self.angular_accel = 0.15
        self.angular_decel = 0.25

        self.last_lin_input = 0.0
        self.last_ang_input = 0.0
        self.input_timeout = 0.18    # seconds: shorter -> release is detected faster

        print("Keyboard controller ready (w/s a/d, space stop).")

    def _read_key(self, timeout=0.01):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                c = sys.stdin.read(1)
                return c
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return None

    def _timer_cb(self):
        key = self._read_key()
        now = time.time()

        # immediate targets on key press
        if key == 'w':
            self.target_lin = self.max_lin
            self.last_lin_input = now
        elif key == 's':
            self.target_lin = -self.max_lin
            self.last_lin_input = now
        elif key == 'a':
            self.target_ang = self.max_ang
            self.last_ang_input = now
        elif key == 'd':
            self.target_ang = -self.max_ang
            self.last_ang_input = now
        elif key == ' ':
            self.target_lin = 0.0
            self.target_ang = 0.0
            self.last_lin_input = 0.0
            self.last_ang_input = 0.0
        elif key == '\x03':  # Ctrl-C
            rclpy.shutdown()
            return

        # clear angular target quickly when no a/d pressed
        if now - self.last_ang_input > self.input_timeout:
            self.target_ang = 0.0

        # clear linear target quickly when no w/s pressed -> auto-brake
        if now - self.last_lin_input > self.input_timeout:
            self.target_lin = 0.0

        # boost angular control while moving forward/back to make w+a responsive
        # scale factor: 1.0 (no move) -> up to 1.6 when at max speed
        lin_factor = min(abs(self.target_lin) / max(self.max_lin, 1e-6), 1.0)
        angular_boost = 1.0 + 0.6 * lin_factor   # increase boost if needed
        desired_ang = self.target_ang * angular_boost

        # ramp linear toward target
        if self.lin < self.target_lin:
            self.lin = min(self.lin + self.linear_accel, self.target_lin)
        elif self.lin > self.target_lin:
            self.lin = max(self.lin - self.linear_decel, self.target_lin)

        # ramp angular toward boosted target
        if self.ang < desired_ang:
            self.ang = min(self.ang + self.angular_accel, desired_ang)
        elif self.ang > desired_ang:
            self.ang = max(self.ang - self.angular_decel, desired_ang)

        # clamp small noise to zero
        if abs(self.lin) < 1e-3 and self.target_lin == 0.0:
            self.lin = 0.0
        if abs(self.ang) < 1e-3 and desired_ang == 0.0:
            self.ang = 0.0

        # publish
        msg = Twist()
        msg.linear.x = float(self.lin)
        msg.angular.z = float(self.ang)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
