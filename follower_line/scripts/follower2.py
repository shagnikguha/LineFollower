#!/usr/bin/env python3

"""line_follower.py: Robot will follow the black line and stop at endpoint after detecting yellow color."""

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp        #proportional constant
        self.ki = ki        #integral constant
        self.kd = kd        #differential constant
        self.w_dist = 0     #desired distance from destination
        self.c_dist = 0     #current distance from destination
        self.Error = 0      #current error
        self.kiError = 0    #error sum
        self.prevError = 0  #previous error
    
    def control(self, w_dist, c_dist):
        self.w_dist = w_dist
        self.c_dist = c_dist
        self.Error = self.w_dist - self.c_dist                      #error calculation
        self.Proportional = self.kp * self.Error                    #proportional out 
        self.kiError = self.kiError + self.Error                    #total error calculation
        self.Integral = self.ki * self.kiError                      #integral out    
        self.Differential = self.kd * (self.Error*self.prevError)   #differential out
        self.prevError = self.Error

        self.Total_out =  self.Proportional+self.Integral+self.Differential

        return self.Total_out
    
class LineFollower:
    def __init__(self):
        self.control = PID(0.003, 0.000001, 0)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect black line
        lower_black = numpy.array([0, 0, 0])
        upper_black = numpy.array([10, 10, 10])
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 20
        mask_black[0:int(search_top), 0:int(w)] = 0
        mask_black[int(search_bot):h, 0:int(w)] = 0

        # Calculate centroid of the black line
        M = cv2.moments(mask_black)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # Control based on line position
            dist_from_cntr = cx - w / 2

            if(abs(dist_from_cntr)>10):
              self.twist.angular.z = (self.control.control(0,dist_from_cntr))
            else:
              self.twist.angular.z = 0
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)

        # Detect yellow color
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 250])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        search_top_y = 299* h / 300
        search_bot_y = 299* h / 300 + 10
        mask_yellow[0:int(search_top_y), 0:int(w)] = 0
        mask_yellow[int(search_bot_y):h, 0:int(w)] = 0

        M = cv2.moments(mask_yellow)
        if M['m00'] > 0:
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1.5)
            self.twist.linear.x = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("Yellow color detected. Stopping the robot.")
            rospy.signal_shutdown("Yellow color detected")


        cv2.imshow("mask_black", mask_black)
        cv2.imshow("mask_yellow", mask_yellow)
        cv2.imshow("output", image)
        cv2.waitKey(3)

rospy.init_node('line_follower')
follower = LineFollower()
rospy.spin()
