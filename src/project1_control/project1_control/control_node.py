import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random

KEY_TIMEOUT    = 0.3
FORWARD_SPEED  = 0.2
TURN_SPEED     = 0.6
TURN_DIST      = 0.3048
FRONT_ARC      = math.radians(30)
ASYM_THRESHOLD = 0.05
COLLISION_DIST = 0.25
LOG_THROTTLE   = 1.0


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_remaining(target, current):
    diff = target - current
    return (diff + math.pi) % (2 * math.pi) - math.pi


class Project1Controller(Node):
    def __init__(self):
        super().__init__('project1_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist,     '/cmd_vel_key', self.key_callback,  10)
        self.create_subscription(Odometry,  '/odom',        self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan',        self.scan_callback, 10)
        self.create_timer(0.05, self.tick)

        # keyboard
        self.last_key_cmd  = Twist()
        self.key_active    = False
        self.key_last_time = 0.0

        # odometry
        self.last_x          = None
        self.last_y          = None
        self.current_yaw     = 0.0
        self.dist_since_turn = 0.0

        # priority 3 — escape 
        self.escape_active     = False
        self.escape_phase      = None  # 'turn' | 'drive'
        self.escape_target_yaw = 0.0
        self.escape_cooldown   = 0.0

        # priority 4 — avoid
        self.avoid_active     = False
        self.avoid_target_yaw = 0.0

        # priority 5 — random turn
        self.random_turn_active     = False
        self.random_turn_target_yaw = 0.0
        self.random_turn_dir        = 1

        # lidar
        self.front_left_min     = float('inf')
        self.front_right_min    = float('inf')
        self.collision_detected = False

        self.get_logger().info('Controller started.')

    def key_callback(self, msg: Twist):
        self.last_key_cmd  = msg
        self.key_last_time = self.get_clock().now().nanoseconds / 1e9
        self.key_active    = (msg.linear.x != 0.0 or msg.angular.z != 0.0)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.last_x is None:
            self.last_x, self.last_y = x, y
            return

        dx = x - self.last_x
        dy = y - self.last_y
        self.dist_since_turn += math.sqrt(dx*dx + dy*dy)
        self.last_x, self.last_y = x, y

    def scan_callback(self, msg: LaserScan):
        left_min  = float('inf')
        right_min = float('inf')
        collision = False

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0.0:
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            if r < COLLISION_DIST:
                collision = True

            angle = msg.angle_min + i * msg.angle_increment
            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            if abs(angle) <= FRONT_ARC:
                if angle >= 0:
                    left_min  = min(left_min,  r)
                else:
                    right_min = min(right_min, r)

        self.collision_detected = collision
        self.front_left_min     = left_min
        self.front_right_min    = right_min

    def tick(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.key_active and (now - self.key_last_time) > KEY_TIMEOUT:
            self.key_active = False

        # priority 2: keyboard
        if self.key_active:
            self.random_turn_active = False
            self.escape_active      = False
            self.avoid_active       = False
            self.dist_since_turn    = 0.0
            self.cmd_pub.publish(self.last_key_cmd)
            return

        left  = self.front_left_min
        right = self.front_right_min

        cmp_left  = left  if not math.isinf(left)  else TURN_DIST * 2
        cmp_right = right if not math.isinf(right) else TURN_DIST * 2
        asymmetric = abs(cmp_left - cmp_right) > ASYM_THRESHOLD

        # priority 3: escape
        if self.collision_detected and not self.escape_active and now > self.escape_cooldown:
            self.avoid_active       = False
            self.random_turn_active = False
            self.dist_since_turn    = 0.0

            escape_angle           = random.uniform(math.radians(150), math.radians(210))
            turn_dir               = random.choice([-1, 1])
            self.escape_target_yaw = self.current_yaw + turn_dir * escape_angle
            self.escape_active     = True
            self.escape_phase      = 'turn'
            self.get_logger().info(
                f'P3 Escape: triggered — turning {math.degrees(escape_angle):.1f}° '
                f'{"CCW" if turn_dir > 0 else "CW"}'
            )

        if self.escape_active:
            if self.escape_phase == 'turn':
                rem = yaw_remaining(self.escape_target_yaw, self.current_yaw)
                if abs(rem) < math.radians(2):
                    self.escape_phase    = 'drive'
                    self.dist_since_turn = 0.0
                    self.get_logger().info('P3 Escape: turn done — driving forward.')
                else:
                    cmd.linear.x  = 0.0
                    cmd.angular.z = TURN_SPEED * (1 if rem > 0 else -1)

            elif self.escape_phase == 'drive':
                if self.dist_since_turn >= TURN_DIST:
                    self.escape_active   = False
                    self.escape_phase    = None
                    self.escape_cooldown = now + 5.0
                    self.dist_since_turn = 0.0
                    self.get_logger().info('P3 Escape: complete.')
                else:
                    cmd.linear.x  = FORWARD_SPEED
                    cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            return

        # priority 4: avoid
        left_close   = left  < TURN_DIST
        right_close  = right < TURN_DIST
        either_close = left_close or right_close

        if either_close and asymmetric and not self.avoid_active and now > self.escape_cooldown:
            avoid_angle = random.uniform(math.radians(30), math.radians(60))
            if cmp_left < cmp_right:
                turn_dir = -1
                self.get_logger().info(f'P4 Avoid: obstacle LEFT ({left:.2f}m) — turning right.')
            else:
                turn_dir = 1
                self.get_logger().info(f'P4 Avoid: obstacle RIGHT ({right:.2f}m) — turning left.')
            self.avoid_target_yaw = self.current_yaw + turn_dir * avoid_angle
            self.avoid_active     = True

        if self.avoid_active:
            rem = yaw_remaining(self.avoid_target_yaw, self.current_yaw)
            if abs(rem) < math.radians(2):
                self.avoid_active = False
                self.get_logger().info('P4 Avoid: complete.')
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = TURN_SPEED * (1 if rem > 0 else -1)
                self.cmd_pub.publish(cmd)
            return

        # priority 5: random turn every 1 ft
        if not self.random_turn_active and self.dist_since_turn >= TURN_DIST:
            angle                       = random.uniform(-math.pi / 12, math.pi / 12)
            self.random_turn_dir        = 1 if angle >= 0 else -1
            self.random_turn_target_yaw = self.current_yaw + angle
            self.random_turn_active     = True
            self.dist_since_turn        = 0.0
            self.get_logger().info(
                f'P5 Random turn: {math.degrees(angle):.1f}° '
                f'-> target yaw {math.degrees(self.random_turn_target_yaw):.1f}°'
            )

        if self.random_turn_active:
            rem = yaw_remaining(self.random_turn_target_yaw, self.current_yaw)
            if abs(rem) < math.radians(2):
                self.random_turn_active = False
                self.get_logger().info('P5 Random turn: complete.')
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = TURN_SPEED * (1 if rem > 0 else -1)
                self.cmd_pub.publish(cmd)
                return

        # priority 6: drive forward
        cmd.linear.x  = FORWARD_SPEED
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Project1Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()