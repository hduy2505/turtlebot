# White Line Follower

## Overview

This script enables a robot to follow a white line on the ground using a camera and ROS (Robot Operating System). The camera captures images, which are processed to detect the white line. The robot adjusts its movement based on the position of the line relative to the center of the image, ensuring it stays on track. The approach leverages image processing techniques with OpenCV to identify the white line and ROS to control the robot's movements.

## Diagram

Here's a high-level diagram of the process:

Camera Image Feed --> Image Processing (OpenCV) --> Calculate Centroid --> Determine Error --> Adjust Robot Movement (ROS)

## Function Descriptions

### 1. `__init__(self)`

- **Description:** Initializes the ROS node, subscribes to the image topic, and sets up a publisher for velocity commands. Also initializes the `Twist` message for controlling the robot's movements.

```python
def __init__(self):
    rospy.init_node('white_line_follower', anonymous=True)
    self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback)
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    self.vel_msg = Twist()
2. image_callback(self, img_data)

    Description: Handles the incoming image data, processes the image to detect the white line, calculates the centroid of the white pixels, and adjusts the robot's velocity based on the position of the centroid relative to the image center. Publishes the velocity commands to the robot.
Main Execution

    Description: Creates an instance of the WhiteLineFollower class and keeps the node running and processing images until manually interrupted. Handles any exceptions during execution.
def image_callback(self, img_data):
    img_np = np.frombuffer(img_data.data, np.uint8)
    img_bgr = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, mask_white = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)
    moments = cv2.moments(mask_white)
    
    if moments['m00'] > 0:
        centroid_x = int(moments['m10'] / moments['m00'])
        centroid_y = int(moments['m01'] / moments['m00'])
        cv2.circle(img_bgr, (centroid_x, centroid_y), 5, (0, 0, 255), -1)
        img_center_x = img_bgr.shape[1] / 2
        error_x = centroid_x - img_center_x
        self.vel_msg.linear.x = 0.05
        self.vel_msg.angular.z = -error_x / 100
        self.vel_pub.publish(self.vel_msg)

    cv2.imshow("Processed Image", img_bgr)
    cv2.waitKey(1)
    Converts the compressed image data to a NumPy array.
    Decodes the NumPy array to a BGR image using OpenCV.
    Converts the BGR image to a grayscale image.
    Applies a binary threshold to create a mask of white pixels.
    Calculates the moments of the binary image to find the centroid of the white pixels.
    If white pixels are detected (moments['m00'] > 0):
        Computes the centroid of the white pixels.
        Draws a small red circle at the centroid for visualization.
        Calculates the horizontal error between the centroid and the center of the image.
        Sets the robot's linear speed to a constant value.
        Sets the robot's angular speed based on the error.
        Publishes the velocity command to the robot.
    Displays the processed image with the centroid for debugging purposes.

Main Execution

python

if __name__ == '__main__':
    try:
        line_follower = WhiteLineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    Creates an instance of the WhiteLineFollower class.
    Keeps the program running and processing images until manually interrupted using rospy.spin().
    Handles any exceptions that occur during the execution of the ROS node.