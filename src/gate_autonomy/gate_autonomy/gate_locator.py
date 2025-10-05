#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from vision_msgs.msg import BoundingBoxArray
import time

class GateLocator(Node):
    def __init__(self):
        super().__init__('gate_locator')

        # Publishers & Subscribers
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.alt_callback, 10)
        self.bb_sub = self.create_subscription(BoundingBoxArray, '/main_camera/detection/bounding_boxes', self.bb_callback, 10)

        # State
        self.current_alt = None
        self.target_alt = -1.5  # target depth, set to-1.2 to stop above orange obstacle
        self.gate_detected = False
        self.gate_x = 0.5        # horizontal position (0-1)
        self.gate_last_seen_time = 0.0
        self.gate_timeout = 1.5  # seconds

        # Controller parameters
        self.k_p = 1.0            # proportional gain for angular correction
        self.k_lat = 0.5          # proportional gain for lateral motion
        self.forward_speed = 1.0  # base forward speed m/s
        self.center_threshold = 0.2  # threshold for considering centered

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def alt_callback(self, msg):
        self.current_alt = msg.data

    def bb_callback(self, msg):
        for box in msg.bounding_boxes:
            if box.label_name == "gate":
                self.gate_detected = True
                self.gate_x = box.x
                self.gate_last_seen_time = time.time()
                self.get_logger().info(f"Gate detected at ({box.x:.2f}, {box.y:.2f})")
                return

    def control_loop(self):
        if self.current_alt is None:
            return

        # Check if gate was seen recently
        if time.time() - self.gate_last_seen_time > self.gate_timeout:
            self.gate_detected = False

        msg = Twist()

        # --- Depth control with proportional gain ---
        if self.current_alt > self.target_alt + 0.05:
            depth_error = self.target_alt - self.current_alt
            k_depth = 1.5  # gain for faster descent
            msg.linear.z = max(min(k_depth * depth_error, 2.0), -2.0)  # clamp
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            self.get_logger().info(f"Descending... Altitude: {self.current_alt:.2f}")
        else:
            msg.linear.z = 0.0  # hold depth

            # --- Gate search & centering ---
            if not self.gate_detected:
                msg.angular.z = 0.7  # rotate to search
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                self.get_logger().info("Searching for gate...")
            else:
                # Gate detected: steer while moving forward
                x_error = self.gate_x - 0.5

                # Angular correction
                msg.angular.z = -self.k_p * x_error

                # Lateral motion proportional to horizontal error (clamped)
                msg.linear.y = max(min(-self.k_lat * x_error, 0.5), -0.5)

                # Forward speed reduces slightly if far from center
                if abs(x_error) < self.center_threshold:
                    msg.linear.x = self.forward_speed
                else:
                    msg.linear.x = self.forward_speed * max(0.3, 1 - abs(x_error)/0.5)

                self.get_logger().info(f"Approaching gate, x_error={x_error:.2f}, angular={msg.angular.z:.2f}, lateral={msg.linear.y:.2f}")

        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GateLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
