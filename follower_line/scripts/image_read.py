#!/usr/bin/env python3

import rospy
import cv2 as cv
import cv_bridge
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image



class Read:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.image_callback)
        #self.img_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_clbk)#sensor_msgs/CompressedImage
        self.img_sub = rospy.Subscriber("/camera_image", Image, self.image_clbk)
        '''
        live = cv.VideoCapture(6)
        while(True):
            isTrue, frame = live.read()
            self.image_clbk(frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break 
         
        '''
    def image_clbk(self, image):
        image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        #hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV_FULL)

        #img = cv.cvtColor(msg, cv.COLOR_BGR2GRAY)
        cv.imshow("video", image)
        cv.waitKey(5)


rospy.init_node("video_reader")
read = Read()
rospy.spin()
            
        