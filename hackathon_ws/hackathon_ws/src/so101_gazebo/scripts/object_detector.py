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
        
        # LOGIC FLAGS
        self.scanning_complete = False
        self.grasp_initiated = False 

        self.get_logger().info('Initializing... ensure Gazebo is PLAYING.')
        time.sleep(2.0)
        
        # STEP 1: Run your original scanning motion
        self.move_to_scanning_pose()

        self.subscription = self.create_subscription(Image, '/d435i/image', self.image_callback, 10)

    def move_to_scanning_pose(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        msg.header.stamp = self.get_clock().now().to_msg()
        
        point = JointTrajectoryPoint()
        # KEPT EXACTLY THE SAME AS YOUR ORIGINAL SCRIPT:
        point.positions = [0.0, -1.0, 1.0, 1.7, -1.7, 0.0]
        point.time_from_start.sec = 4
        msg.points.append(point)
        
        self.get_logger().info('Sending command to ALL 6 joints...')
        for _ in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
            
        self.get_logger().info('Waiting for physical motion...')
        time.sleep(5.0) 
        
        self.scanning_complete = True
        self.get_logger().info('Motion window closed. Starting detection.')

    def image_callback(self, msg):
        if not self.scanning_complete or self.grasp_initiated:
            return 

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 60]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(cv_image, (cX, cY), 7, (0, 255, 0), -1)
                    
                    self.get_logger().info(f'TARGET at: {cX}, {cY} - INITIATING GRASP!')
                    self.grasp_initiated = True 
                    self.execute_grasp()

        cv2.imshow("Grasping View", cv_image)
        cv2.waitKey(1)

    def execute_grasp(self):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        msg.header.stamp = self.get_clock().now().to_msg()

        # --- YOUR ORIGINAL PICK SEQUENCE (NO CHANGES) ---
        
        # STEP 1: arive
        lift_back = JointTrajectoryPoint()
        lift_back.positions = [-0.052,0.198,1.562,-1.658,-1.657,1.745] 
        lift_back.time_from_start.sec = 3
        msg.points.append(lift_back)

        # STEP 2: grasp 
        hover = JointTrajectoryPoint()
        hover.positions = [-0.031,0.991,0.393,-1.389,-1.567,0.386] 
        hover.time_from_start.sec = 6
        msg.points.append(hover)

        # STEP 3: lift
        dive = JointTrajectoryPoint()
        dive.positions = [-0.093,0.009,0.886,-0.923,-1.657,0.365] 
        dive.time_from_start.sec = 7
        msg.points.append(dive)

        # STEP 4: rotate
        retract = JointTrajectoryPoint()
        retract.positions = [1.629,0.009,0.886,-0.923,-1.657,0.365] 
        retract.time_from_start.sec = 9
        msg.points.append(retract)

        # STEP 5: pour 
        pivot = JointTrajectoryPoint()
        pivot.positions = [1.629,0.009,0.886,-0.923,0.426,0.365] 
        pivot.time_from_start.sec = 11
        msg.points.append(pivot)

        # STEP 6: stop
        lower = JointTrajectoryPoint()
        lower.positions = [1.629,0.009,0.886,-0.923,-1.657,0.365] 
        lower.time_from_start.sec = 12
        msg.points.append(lower)

        # STEP 7: keep back
        release = JointTrajectoryPoint()
        release.positions = [-0.031,0.745,0.685,-1.425,-1.657,0.365] 
        release.time_from_start.sec = 16
        msg.points.append(release)

        # STEP 8: release
        retract_after = JointTrajectoryPoint()
        retract_after.positions = [-0.052,0.198,1.562,-1.658,-1.657,1.745] 
        retract_after.time_from_start.sec = 18
        msg.points.append(retract_after)

        # STEP 9: return
        return_home = JointTrajectoryPoint()
        return_home.positions = [0.031,-1.745,1.690,0.726,-1.476,1.745]
        return_home.time_from_start.sec = 20
        msg.points.append(return_home)

        self.joint_pub.publish(msg)
        self.get_logger().info('Executing Full Cycle: Pick -> Pivot 90 -> Place -> Release -> Home')

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