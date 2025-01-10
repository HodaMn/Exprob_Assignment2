#!/usr/bin/env python3

import roslib
import rospy
import time
import math
import sys
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage, CameraInfo


class camera_stabilizer:
    def __init__(self):
        rospy.init_node('camera_stabilizer_node')
        
        # Initialize points for camera and marker centers
        self.camera_center = Point()
        self.current_marker_center = Point()
        self.detected_marker_ids = []
        self.marker_positions = {}  # Holds (x, y) coordinates for each detected marker
        self.current_id = 0
        self.collecting_data = True  # Flag to manage the data collection phase
        self.marker_reached = False
        self.active_marker = 0
                
        # Publishers
        self.annotated_image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/robot/camera1/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        rospy.Subscriber("/robot/camera1/camera_info", CameraInfo, self.camera_info_handler, queue_size=1)        
        rospy.Subscriber("/marker/id_number", Int32, self.id_handler, queue_size=1)        
        rospy.Subscriber("/marker/center_loc", Point, self.marker_center_handler, queue_size=1)
        
    def camera_info_handler(self, msg):
        """Determine the center of the camera frame based on camera info."""
        self.camera_center.x = msg.height / 2
        self.camera_center.y = msg.width / 2
    
    def id_handler(self, msg):
        """Update the current marker ID based on incoming data."""
        self.current_id = msg.data        
    
    def marker_center_handler(self, msg):
        """Update the marker's center position and store it if it's a new marker."""
        self.current_marker_center.x = msg.x
        self.current_marker_center.y = msg.y
        if self.current_id and self.current_id not in self.detected_marker_ids:
            self.marker_positions[self.current_id] = (msg.x, msg.y)
                                                    
    def image_callback(self, msg):
        """Process the incoming image, control robot movement, and publish annotated images."""
        # Convert the compressed image data to an OpenCV-compatible format
        np_arr = np.frombuffer(msg.data, np.uint8)  # Changed from np.fromstring to np.frombuffer        
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode the image using OpenCV
        
        velocity_cmd = Twist()        
        
        if self.collecting_data:        
            if len(self.detected_marker_ids) < 7: 
                # Rotate the camera to find all markers
                velocity_cmd.linear.x = 0 
                velocity_cmd.angular.z = 0.7
                if self.current_id and self.current_id not in self.detected_marker_ids:
                    self.detected_marker_ids.append(self.current_id)
                    self.detected_marker_ids.sort()
                    rospy.loginfo(f"Detected marker IDs: {self.detected_marker_ids}")                                
            
            else:
                # Data collection complete, switch to stabilization mode
                self.collecting_data = False
                rospy.loginfo("All markers detected. Entering stabilization mode.")
                
        else:
            if self.detected_marker_ids:
                self.active_marker = self.detected_marker_ids[0]
                target_x = self.current_marker_center.x
                target_y = self.current_marker_center.y
                rospy.loginfo(f"Target X: {target_x}, Camera Center X: {self.camera_center.x}, Current Marker ID: {self.active_marker}")
                
                # Check if the active marker is centered within a threshold
                if abs(self.camera_center.x - target_x) < 10 and self.current_id == self.active_marker:
                    self.marker_reached = True
                    velocity_cmd.angular.z = 0
                    rospy.loginfo(f"Marker {self.active_marker} is centered.")
                    
                    # Highlight the marker on the image
                    cv2.circle(image, (int(target_x), int(target_y)), 25, (0, 255, 0), 4)
                    
                    # Publish the annotated image
                    annotated_image = CompressedImage()
                    annotated_image.header.stamp = rospy.Time.now()
                    annotated_image.format = "jpeg"
                    annotated_image.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
                    
                    self.annotated_image_pub.publish(annotated_image)                    
                    
                if self.marker_reached:
                    # Remove the reached marker and reset the flag
                    self.detected_marker_ids.pop(0)
                    self.marker_reached = False
                    velocity_cmd.angular.z = 0
                    
                elif self.camera_center.x > target_x and self.current_id == self.active_marker:
                    # Adjust camera to the left
                    velocity_cmd.angular.z = 0.3
                    rospy.loginfo("Adjusting camera to the left.")
                    
                elif self.camera_center.x < target_x and self.current_id == self.active_marker:
                    # Adjust camera to the right
                    velocity_cmd.angular.z = -0.3
                    rospy.loginfo("Adjusting camera to the right.")
                else: 
                    # Continue rotating to find the marker
                    velocity_cmd.angular.z = 0.5
                    rospy.loginfo("Continuing camera rotation.")
                    
            else:
                # All markers have been processed
                velocity_cmd.angular.z = 0
                rospy.loginfo("All markers have been stabilized.")
                    
        self.cmd_vel_pub.publish(velocity_cmd)    
    
    
def main():
    stabilizer = camera_stabilizer()
    rospy.spin()
    
if __name__ == '__main__':
    main()


