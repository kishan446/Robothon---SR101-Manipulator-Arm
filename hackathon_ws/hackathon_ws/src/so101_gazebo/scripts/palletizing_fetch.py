import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import cv2
import numpy as np
import time

class PalletizingNode(Node):
    def __init__(self):
        super().__init__('palletizing_node')
        
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.bridge = CvBridge()
        
        # State Management
        self.scanning_complete = False
        self.target_locked = False 

        self.get_logger().info('Initializing... ensure Gazebo is PLAYING.')
        time.sleep(2.0)
        
        # 1. Start by moving to the overview pose
        self.move_to_scanning_pose()

        self.subscription = self.create_subscription(Image, '/d435i/image', self.image_callback, 10)

    def move_to_scanning_pose(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        msg.header.stamp = self.get_clock().now().to_msg()
        
        point = JointTrajectoryPoint()
        # Adjusted: Pan toward cup, Lift up, Flex down, Roll at 0 (straight)
        # [pan, lift, flex, w_flex, w_roll, gripper]
        point.positions = [-0.7, -0.6, 0.8, 1.3, 0.0, 0.0]
        point.time_from_start.sec = 4
        msg.points.append(point)
        
        self.get_logger().info('Moving to Scanning Pose...')
        for _ in range(5):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
            
        time.sleep(4.0) 
        self.scanning_complete = True

    def image_callback(self, msg):
        # Only run detection if we haven't locked onto the target yet
        if not self.scanning_complete or self.target_locked:
            return 

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 60]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 800: # Threshold for detection
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(cv_image, (cX, cY), 7, (0, 255, 0), -1)
                    
                    self.get_logger().info(f'TARGET LOCKED at {cX}, {cY}! Initiating Fetch...')
                    self.target_locked = True # Stop calling this callback
                    self.execute_pick()

        cv2.imshow("Grasping View", cv_image)
        cv2.waitKey(1)

    def execute_pick(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        msg.header.stamp = self.get_clock().now().to_msg()

        # Point 1: Reach the cup with gripper OPEN (0.5)
        reach = JointTrajectoryPoint()
        reach.positions = [-0.7, 0.1, 1.1, 1.4, 0.0, 0.5] 
        reach.time_from_start.sec = 3
        msg.points.append(reach)

        # Point 2: Stay at the cup but CLOSE gripper (0.0)
        # We add 2 seconds to the timer so it happens AFTER the reach
        grasp = JointTrajectoryPoint()
        grasp.positions = [-0.7, 0.1, 1.1, 1.4, 0.0, 0.0] 
        grasp.time_from_start.sec = 5
        msg.points.append(grasp)

        self.get_logger().info('Autonomous Fetch Sequence Started...')
        self.joint_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = PalletizingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()