#!/usr/bin/env python3

# Import necessary libraries
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class WhiteLineFollower:
    def __init__(self):
        # Initialize the ROS node with the name 'white_line_follower'
        rospy.init_node('white_line_follower', anonymous=True)
        
        # Subscribe to the topic '/raspicam_node/image/compressed' to receive compressed image data
        self.image_sub = rospy.Subscriber(
            "/raspicam_node/image/compressed", 
            CompressedImage, 
            self.image_callback
        )
        
        # Create a publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            '/cmd_vel', 
            Twist, 
            queue_size=10
        )
        
        # Initialize the Twist message for controlling robot's linear and angular velocities
        self.vel_msg = Twist()
        
    def image_callback(self, img_data):
        # Convert the compressed image data to a numpy array
        img_np = np.frombuffer(img_data.data, np.uint8)
        
        # Decode the numpy array to a BGR image using OpenCV
        img_bgr = cv2.imdecode(img_np, cv2.IMREAD_COLOR)

        # Convert the BGR image to a grayscale image
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        
        # Apply a binary threshold to the grayscale image to create a mask of white pixels
        _, mask_white = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)

        # Calculate the moments of the binary image
        moments = cv2.moments(mask_white)
        
        if moments['m00'] > 0:
            # Compute the x and y coordinates of the centroid of the white pixels
            centroid_x = int(moments['m10'] / moments['m00'])
            centroid_y = int(moments['m01'] / moments['m00'])

            # Draw a small red circle at the centroid on the original image for visualization
            cv2.circle(img_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

            # Calculate the horizontal error between the centroid and the center of the image
            img_center_x = img_bgr.shape[1] / 2
            error_x = centroid_x - img_center_x
            
            # Set the robot's linear speed to a constant value of 0.05 m/s
            self.vel_msg.linear.x = 0.05
            
            # Set the robot's angular speed based on the error
            self.vel_msg.angular.z = -error_x / 100

            # Publish the velocity command to the robot
            self.vel_pub.publish(self.vel_msg)

        # Display the processed image with the centroid for debugging purposes
        cv2.imshow("Processed Image", img_bgr)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # Create an instance of the WhiteLineFollower class
        line_follower = WhiteLineFollower()
        
        # Keep the program running and processing images until manually interrupted
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
