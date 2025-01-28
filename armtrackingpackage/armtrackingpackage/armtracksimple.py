import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
class ArmTrackingNode(Node):
    def __init__(self):
        super().__init__('armtracking_node')
        self.elbow_publisher_ = self.create_publisher(Float64, 'elbow_angle', 10)
        self.trajectory_publisher_ = self.create_publisher(JointTrajectory, 'kinect_position_commands', 10)
        self.joint_state_subscription_ = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.current_joint_positions = [0.0] * 6  # Default values for 6 joints
        self.first_joint_state_received = False  # Add this in the __init__ method

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            rclpy.shutdown()
        self.pose = mp.solutions.pose.Pose(min_detection_confidence=0.6, min_tracking_confidence=0.6)

    def joint_state_callback(self, msg):
        if not self.first_joint_state_received:
            self.get_logger().info("Oooh first run.")
            valid_positions = [pos for pos in msg.position if pos is not None]
            if len(valid_positions) == len(msg.position):  # Ensure all positions are valid
                self.get_logger().info("Oooh positions valid.")
                joint_name_map = {name: idx for idx, name in enumerate(msg.name)}
                for i, joint_name in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
                    if joint_name in joint_name_map:
                        self.current_joint_positions[i] = msg.position[joint_name_map[joint_name]]
                        self.get_logger().error(f"Position {i}: {self.current_joint_positions[i]}")
                self.first_joint_state_received = True
                self.get_logger().error("Processed first valid joint state message.")

    def calculate_angle(self, v1, v2, normal):
        """Calculate the signed angle between two vectors using the dot and cross products."""
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)
        cos_theta = dot_product / (norm_v1 * norm_v2)
        angle = np.arccos(cos_theta)
        
        # Calculate the cross product to determine the sign of the angle
        cross_product = np.cross(v1, v2)
        if np.dot(cross_product, normal) < 0:
            angle = -angle
        
        return np.degrees(angle)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error: Could not read frame.")
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb_frame)
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            h, w, _ = frame.shape

            left_shoulder = np.array([landmarks[11].x * w, landmarks[11].y * h, landmarks[11].z * w])
            left_elbow = np.array([landmarks[13].x * w, landmarks[13].y * h, landmarks[13].z * w])
            left_wrist = np.array([landmarks[15].x * w, landmarks[15].y * h, landmarks[15].z * w])

            upper_arm = left_elbow - left_shoulder
            lower_arm = left_wrist - left_elbow

            normal = np.cross(upper_arm, lower_arm)

            elbow_angle = self.calculate_angle(upper_arm, lower_arm, normal) - 40

            # Prepare and publish the trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.header.frame_id = "base_link"
            trajectory_msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = self.current_joint_positions[:]  # Copy current positions
            trajectory_point.positions[2] = np.radians(elbow_angle)  # Replace joint 5 with the elbow angle in radians
            trajectory_point.time_from_start.sec = 1
            trajectory_point.time_from_start.nanosec = 0
            trajectory_msg.points.append(trajectory_point)

            self.trajectory_publisher_.publish(trajectory_msg)
            # Annotate and display the frame
            annotated_frame = frame.copy()
            left_arm_landmarks = [11, 13, 15]
            for idx in left_arm_landmarks:
                landmark = landmarks[idx]
                cx, cy = int(landmark.x * w), int(landmark.y * h)
                cv2.circle(annotated_frame, (cx, cy), 5, (0, 255, 0), -1)
                confidence_score = landmark.visibility
                cv2.putText(annotated_frame, f'{confidence_score:.2f}', (cx, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            connections_left = [(11, 13), (13, 15)]
            for connection in connections_left:
                start_idx, end_idx = connection
                start_landmark = landmarks[start_idx]
                end_landmark = landmarks[end_idx]
                start_point = (int(start_landmark.x * w), int(start_landmark.y * h))
                end_point = (int(end_landmark.x * w), int(end_landmark.y * h))
                cv2.line(annotated_frame, start_point, end_point, (0, 255, 0), 2)

            cv2.putText(annotated_frame, f'Elbow Angle: {elbow_angle:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow('Webcam Arm Tracking', annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArmTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
