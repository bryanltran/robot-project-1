import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

KEY_TIMEOUT = 0.3
FORWARD_SPEED = 0.2
BUMP_DIST = 0.15  # treat anything this close as a bumper hit


class Project1Controller(Node):
    def __init__(self):
        super().__init__('project1_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_key', self.key_callback, 10)

        self.create_timer(0.05, self.tick)

        self.halt_active = False
        self.last_key_cmd = Twist()
        self.key_active = False
        self.key_last_time = 0.0
        self.forward_speed = FORWARD_SPEED

        self.get_logger().info('Project1 Controller started.')

    def scan_callback(self, msg: LaserScan):
        import math
        if any(r < BUMP_DIST for r in msg.ranges
               if not math.isnan(r) and not math.isinf(r) and r > 0.0):
            if not self.halt_active:
                self.get_logger().warn('Bumper hit - halting!')
            self.halt_active = True

    def key_callback(self, msg: Twist):
        self.last_key_cmd = msg
        self.key_last_time = self.get_clock().now().nanoseconds / 1e9
        self.key_active = (
            msg.linear.x != 0.0 or
            msg.linear.y != 0.0 or
            msg.angular.z != 0.0
        )

    def tick(self):
        cmd = Twist()

        now = self.get_clock().now().nanoseconds / 1e9
        if self.key_active and (now - self.key_last_time) > KEY_TIMEOUT:
            self.key_active = False

        # Priority 1: Halt
        if self.halt_active:
            self.cmd_pub.publish(cmd)
            return

        # Priority 2: Keyboard
        if self.key_active:
            self.cmd_pub.publish(self.last_key_cmd)
            return

        # Priority 6: Drive forward
        cmd.linear.x = self.forward_speed
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Project1Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()