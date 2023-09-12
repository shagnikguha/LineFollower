#!/usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
import cv_bridge


class Reader:
    def __init__(self, x):
        self.bridge = cv_bridge.CvBridge()
        self.cam_pub = rospy.Publisher('/camera_image', Image, queue_size=10)
        #rate = rospy.Rate(10)
        live = cv.VideoCapture(x)
        while not rospy.is_shutdown():
            isTrue, frame = live.read()
            if not isTrue:
                rospy.logerr("UNABLE TO CAPTURE FRAME FROM CAMERA")
            
            self.data(frame)
            if cv.waitKey(1) &0xFF == ord('q'):
                break
            if rospy.is_shutdown():
                live.release()
                break

    def data(self, frame):
        cv.flip(frame, 1) 
        image=self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.cam_pub.publish(image)

if __name__ == '__main__':
    rospy.init_node("cam_publisher", anonymous=False)
    cam_port = 6                                #change value for respective camera port
    read = Reader(cam_port)  
    rospy.spin()              

        
