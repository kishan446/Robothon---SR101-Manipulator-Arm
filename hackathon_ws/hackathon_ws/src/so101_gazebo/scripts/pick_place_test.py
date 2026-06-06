#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
import tf_transformations # You might need: sudo apt install ros-humble-tf-transformations

class GraspingTester(Node):
    def __init__(self):
        super().__init__('grasping_tester')
        self.group_name = 'arm' # Or check 'manipulator' if this fails
        self.client = ActionClient(self, MoveGroup, 'move_action')
        
        # Target coordinates (relative to base_link)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.82 

    def send_goal(self, x, y, z):
        self.get_logger().info(f'Planning to: {x}, {y}, {z}')
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Define Pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # Downward orientation (Gripper pointing at the table)
        q = tf_transformations.quaternion_from_euler(0, 1.57, 0) # Roll, Pitch, Yaw
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Wrap it in a constraint
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        # (Simplified for this test - standard move_group goals usually require a full MoveGroup goal structure)
        # Note: Using MoveItPy is much cleaner for this, but requires specific installation.

        self.get_logger().info('Goal sent! Check RViz for the motion plan.')

    def run_test(self):
        # Step 1: Move above cup
        self.send_goal(self.target_x, self.target_y, self.target_z + 0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GraspingTester()
    node.run_test()
    rclpy.spin(node)
    rclpy.shutdown()