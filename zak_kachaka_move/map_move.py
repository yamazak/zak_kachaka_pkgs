import rclpy
from rclpy.node import Node
import rclpy.action
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_client')
        self.client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, pose_stamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result

def create_pose_stamped():
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.header.stamp = rclpy.time.Time().to_msg()

    pose_stamped.pose.position.x = 1.7
    pose_stamped.pose.position.y = 0.9
    pose_stamped.pose.position.z = 0.0

    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    return pose_stamped

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Client()
    pose_stamped = create_pose_stamped()
    node.send_goal(pose_stamped)
    rclpy.shutdown()

if __name__ == "__main__":
    main()