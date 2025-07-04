#!/usr/bin/env python3

import rospy
import cv2
import numpy as np # Already imported
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String # To publish the color
import tf2_ros
import tf2_geometry_msgs
from collections import deque # For storing detected objects
# from math import dist # No longer needed if using numpy.linalg.norm

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS Publishers
        self.object_pose_pub = rospy.Publisher('/detected_object_pose', PoseStamped, queue_size=10)
        self.object_color_pub = rospy.Publisher('/detected_object_color', String, queue_size=10)


        # ROS Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        # Define HSV color ranges for Red, Green, Blue
        self.color_ranges = {
            "red": ([0, 100, 100], [10, 255, 255]),      # Red (lower hue)
            "red2": ([170, 100, 100], [180, 255, 255]),  # Red (upper hue - wrap around)
            "green": ([50, 100, 100], [70, 255, 255]),
            "blue": ([100, 100, 100], [120, 255, 255]),
        }

        # Store detected objects to avoid processing the same one multiple times quickly
        # Stores (color, PoseStamped)
        self.detected_objects_queue = deque(maxlen=5) # Max 5 objects in queue

        rospy.loginfo("Object Detector Node Initialized.")

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            rospy.loginfo("Camera Info Received.")

    def image_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(e)

    def get_3d_coordinates(self, u, v):
        if self.depth_image is None or self.camera_info is None:
            return None

        depth = self.depth_image[v, u] / 1000.0  # Convert mm to meters

        if depth == 0 or np.isnan(depth) or depth > 5.0:
            return None

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        x_c = (u - cx) * depth / fx
        y_c = (v - cy) * depth / fy
        z_c = depth

        return np.array([x_c, y_c, z_c])

    def detect_and_publish_objects(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return

        hsv_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        display_image = self.rgb_image.copy()

        for color_name, (lower, upper) in self.color_ranges.items():
            if color_name == "red":
                mask1 = cv2.inRange(hsv_image, np.array(self.color_ranges["red"][0]), np.array(self.color_ranges["red"][1]))
                mask2 = cv2.inRange(hsv_image, np.array(self.color_ranges["red2"][0]), np.array(self.color_ranges["red2"][1]))
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                lower_bound = np.array(lower)
                upper_bound = np.array(upper)
                mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            min_area = 500 # Adjust this based on your object size in pixels
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_area:
                    M = cv2.moments(contour)
                    if M["m00"] == 0:
                        continue

                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    object_3d_camera = self.get_3d_coordinates(cx, cy)

                    if object_3d_camera is not None:
                        object_pose_camera = PoseStamped()
                        object_pose_camera.header.frame_id = self.camera_info.header.frame_id
                        object_pose_camera.header.stamp = rospy.Time.now()
                        object_pose_camera.pose.position.x = object_3d_camera[0]
                        object_pose_camera.pose.position.y = object_3d_camera[1]
                        object_pose_camera.pose.position.z = object_3d_camera[2]
                        object_pose_camera.pose.orientation.w = 1.0

                        try:
                            transform = self.tf_buffer.lookup_transform(
                                'base_link', object_pose_camera.header.frame_id,
                                rospy.Time(0), rospy.Duration(1.0)
                            )
                            object_pose_robot_frame = tf2_geometry_msgs.do_transform_pose(object_pose_camera, transform)
                            
                            is_new_object = True
                            for (prev_color, prev_pose) in self.detected_objects_queue:
                                if prev_color == color_name:
                                    # Calculate Euclidean distance using numpy.linalg.norm
                                    current_pos_2d = np.array([object_pose_robot_frame.pose.position.x, object_pose_robot_frame.pose.position.y])
                                    prev_pos_2d = np.array([prev_pose.pose.position.x, prev_pose.pose.position.y])
                                    
                                    if np.linalg.norm(current_pos_2d - prev_pos_2d) < 0.03: # 3cm tolerance
                                        is_new_object = False
                                        break
                            
                            if is_new_object:
                                rospy.loginfo("Detected {} object at: x={:.3f}, y={:.3f}, z={:.3f} in base_link frame".format(
                                    color_name.upper(),
                                    object_pose_robot_frame.pose.position.x,
                                    object_pose_robot_frame.pose.position.y,
                                    object_pose_robot_frame.pose.position.z
                                ))
                                self.object_pose_pub.publish(object_pose_robot_frame)
                                self.object_color_pub.publish(String(data=color_name))
                                self.detected_objects_queue.append((color_name, object_pose_robot_frame))

                                cv2.circle(display_image, (cx, cy), 7, (0, 255, 0), -1)
                                cv2.putText(display_image, color_name.upper(), (cx - 20, cy - 20),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                cv2.drawContours(display_image, [contour], -1, (0, 255, 0), 2)
                                
                                # Process only one new object per frame for simplicity.
                                # If you expect multiple objects of different colors to appear at once,
                                # remove this 'break' and potentially store all detected objects from this frame.
                                break 


                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                            rospy.logerr("TF Error transforming object pose: {}".format(e))
            
        # Optional: Display images for debugging
        # cv2.imshow("RGB Image with Detections", display_image)
        # cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.detect_and_publish_objects()
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass