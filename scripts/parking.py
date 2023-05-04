#!/usr/bin/env python3


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from std_msgs.msg import String

class ParkingController:
    def __init__(self):
        rospy.init_node('parking_controller')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/arm_camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('/parking_controller/detected_circle', Image, queue_size=10)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.command_listener = rospy.Subscriber("/park_command", String, self.start_parking)
        self.circles = None
        self.parking_mode = False
        rospy.loginfo("INIT")


    def start_parking(self, msg):
        if msg.data == "parking_on":
            self.parking_mode = True

    def image_callback(self, data):
        if (self.parking_mode):
            rospy.loginfo("DOBIM SLIKO")
            # Convert the image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert the image to grayscale and apply a blur to reduce noise
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (29, 29), 0)

            # Use the HoughCircles function to detect circles in the image
            self.circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=2, minDist=50,
                                            param1=50, param2=30, minRadius=150, maxRadius=200)

            
            x_sum = 0
            y_sum = 0
            radius_sum = 0

            # Draw circle on the image if detected
            if self.circles is not None:
                for circle in self.circles[0]:
                    #rospy.loginfo("raidus: " + str(circle[2]))
                    center = (int(circle[0]), int(circle[1]))
                    x_sum += int(circle[0])
                    y_sum += int(circle[1])
                    radius = int(circle[2])
                    radius_sum += radius
                    #cv2.circle(cv_image, center, radius, (0, 0, 255), 2)
                
                target_x = int(x_sum/len(self.circles[0]))
                target_y = int(y_sum/len(self.circles[0]))
                target_radius = int(radius_sum/len(self.circles[0]))
                #rospy.loginfo("AVG raidus: " + str(target_radius))
                center = (target_x, target_y)
                cv2.circle(cv_image, center, target_radius, (0, 0, 255), 2)
                twist_msg = Twist() 
                rospy.loginfo("target x: " + str(target_x))
                rospy.loginfo("half x: " + str(cv_image.shape[1] / 2))
                if target_x < (cv_image.shape[1] / 2) - 20:
                    # Publish a Twist message to move the robot towards the center of the circle
                    rospy.loginfo("Rotating right!")
                    twist_msg.angular.z =  0.1
                elif target_x > (cv_image.shape[1] / 2) + 20:
                    twist_msg.angular.z =  -0.1
                        #elif distance > 0.2:
                            #twist_msg.linear.x = 0.1
                        #else:
                    rospy.loginfo("Rotating left!")
                        #self.cmd_vel_pub.publish(twist_msg)
                else:
                    twist_msg.angular.z =  0.0
                    #rospy.loginfo("target y: " + str(target_y))
                    #rospy.loginfo("half x: " + str(cv_image.shape[0] / 2))
                    if target_y < (cv_image.shape[0] * 1 / 1):
                        twist_msg.linear.x = 0.1
                        rospy.loginfo("Going forward!")
                    else:
                        twist_msg.linear.x = 0.0
                        rospy.loginfo("Stop!")

                self.cmd_vel_pub.publish(twist_msg)
            # Publish the modified image with the detected circle
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header = Header(stamp=rospy.Time.now())
            self.image_pub.publish(image_msg)

        




    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.loginfo("TEČE")
            

            rate.sleep()

if __name__ == '__main__':
    controller = ParkingController()
    controller.run()
