import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
import tf_transformations
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import pygame

odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

class Follower(Node):
    def __init__(self) -> None:
        super().__init__("move")
        self._publisher = self.create_publisher(Twist, "/kachaka/manual_control/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._odom_subscriber = self.create_subscription(Odometry, "/kachaka/odometry/odometry", self._odom_callback, odom_qos)
        self._timer = self.create_timer(0.5, self._timer_callback)
        self._cmd_vel = Twist()
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0

    def _timer_callback(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform("home", "base_link", rclpy.time.Time())
            self._x = transform.transform.translation.x
            self._y = transform.transform.translation.y
            q = transform.transform.rotation
            roll, pitch, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self._th = yaw
            self.get_logger().info(f"odom: x={self._x:.2f}, y={self._y:.2f}, th={self._th:.2f}")
        except Exception as e:
            self.get_logger().warn("failuer on getting tf") 
    
    # 試行錯誤のあと。もはや不要
    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        #self._x = msg.pose.pose.position.x
        #self._y = msg.pose.pose.position.y
        #self._th = msg.pose.pose.orientation.z

        #tpose = PoseStamped()
        #tpose.header.frame_id = "odom"
        #tpose.header.stamp = self.get_clock().now().to_msg()
        #tpose.pose.position.x = msg.pose.pose.position.x
        #tpose.pose.position.y = msg.pose.pose.position.y
        #tpose.pose.orientation.w = 1.0

        #tpose = TransformStamped()
        #tpose.header.frame_id = "odom"
        #tpose.header.stamp = self.get_clock().now().to_msg()
        #tpose.transform.translation.x = msg.pose.pose.position.x
        #tpose.transform.translation.y = msg.pose.pose.position.y
        #tpose.transform.rotation.w = 1.0

    """
        pose = self.tf_buffer.transform(tpose, "map", timeout=rclpy.duration.Duration(seconds=1.0))

        self._x = pose.pose.position.x
        self._y = pose.pose.position.y
        self._th = tf_transformations.eular_from_quaternion([
            pose.pose.oriantation.x,
            pose.pose.oriantation.y,
            pose.pose.oriantation.z,
            pose.pose.oriantation.w,
            ])[2]
    """

    def move_once(self, v, w):
        #self.get_logger().info(f"move_once: v={v:.2f}, w={w:.2f}")
        self._cmd_vel.linear.x = v
        self._cmd_vel.angular.z = w
        self._publisher.publish(self._cmd_vel)

    def get_pose(self):
        return [self._x, self._y, self._th]


def main(args=None):
    pygame.init()
    font = pygame.font.SysFont(None, 36)
    screen = pygame.display.set_mode((400, 150))
        
    rclpy.init(args=args)
    follower = Follower()

    # 最初のうちは指令が届かない場合があるので、所定の距離移動するまでゆっくり動かす
    while True:
        follower.move_once(0.05, 0.0)
        rclpy.spin_once(follower, timeout_sec=1)
        odom = follower.get_pose()
        if odom[0] > 0.1:
            break

    follower.get_logger().info("next moving phase")

    # なんか適当な動き
    for i in range(200):
        follower.move_once(0.3, 0.3)
        rclpy.spin_once(follower)

    for i in range(200):
        follower.move_once(0.3, -0.3)
        rclpy.spin_once(follower)
    
    # 別のプロセスから動作指令を受けて動き出してしまう場合があるので、
    # 終了のキー操作を受けるまでは停止命令を送り続ける
    running = True
    while running:
        screen.fill((0, 0, 0))
        screen.blit(font.render("push q when finishing", True, (255, 255, 255)), (60, 60))
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                running = False

        follower.move_once(0.0,  0.0)
        rclpy.spin_once(follower)

    pygame.quit()
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()