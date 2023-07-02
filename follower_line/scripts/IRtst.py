#!/usr/bin/env python3

'''no syntax error. issues with gazebo plugin for IR sensor(ig)'''
''' from team --> #define BLACK 1  #define WHITE 0                     (logically black should be 0)'''
'''Sensors recieved values. need to find value for white and black'''

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        self.ir1_detect = 0
        self.ir2_detect = 0
        self.ir3_detect = 0
        self.ir4_detect = 0
        self.ir5_detect = 0
        self.ir6_detect = 0
        self.ir7_detect = 0
        self.ir8_detect = 0

        self.ir1_sub = rospy.Subscriber("/ir_sensor_0", Range, self.ir1Callback)
        self.ir2_sub = rospy.Subscriber("/ir_sensor_1", Range, self.ir2Callback)
        self.ir3_sub = rospy.Subscriber("/ir_sensor_2", Range, self.ir3Callback)
        self.ir4_sub = rospy.Subscriber("/ir_sensor_3", Range, self.ir4Callback)
        self.ir5_sub = rospy.Subscriber("/ir_sensor_4", Range, self.ir5Callback)
        self.ir6_sub = rospy.Subscriber("/ir_sensor_5", Range, self.ir6Callback)
        self.ir7_sub = rospy.Subscriber("/ir_sensor_6", Range, self.ir7Callback)
        self.ir8_sub = rospy.Subscriber("/ir_sensor_7", Range, self.ir8Callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.left_mot = 0
        self.right_mot = 0
        self.ROTATE_SPEED = 100
        self.MAX_SPEED = 200

    def ir1Callback(self, msg):
        ir1_detect = msg.range
        #rospy.loginfo(ir1_detect)
        self.sensorCallback()

    def ir2Callback(self, msg):
        ir2_detect = msg.range
        #rospy.loginfo(ir2_detect)
        self.sensorCallback()

    def ir3Callback(self, msg):
        ir3_detect = msg.range
        #rospy.loginfo(ir3_detect)
        self.sensorCallback()

    def ir4Callback(self, msg):
        ir4_detect = msg.range
        rospy.loginfo(ir4_detect)
        #self.sensorCallback()

    def ir5Callback(self, msg):
        ir5_detect = msg.range
        #rospy.loginfo(ir5_detect)
        self.sensorCallback()

    def ir6Callback(self, msg):
        ir6_detect = msg.range
        #rospy.loginfo(ir6_detect)
        self.sensorCallback()

    def ir7Callback(self, msg):
        ir7_detect = msg.range
        #rospy.loginfo(ir7_detect)
        self.sensorCallback()

    def ir8Callback(self, msg):
        ir8_detect = msg.range
        #rospy.loginfo(ir8_detect)
        self.sensorCallback()

    def sensorCallback(self):
        # Moving Straight Forward at Full Speed
        if self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 1 and self.ir5_detect == 1 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(self.MAX_SPEED, self.MAX_SPEED)

        # Turning Sharp Right
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 1:
            self.setMotors(self.MAX_SPEED, -(0.75 * self.MAX_SPEED))

        # Turning Sharp Left
        elif self.ir1_detect == 1 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(-(0.75 * self.MAX_SPEED), self.MAX_SPEED)

        # Turning 90 degree Left
        elif self.ir1_detect == 1 and self.ir2_detect == 1 and self.ir3_detect == 1 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(-(0.25 * self.MAX_SPEED), self.MAX_SPEED)

        # Turning 90 degree Right
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 1 and self.ir7_detect == 1 and self.ir8_detect == 1:
            self.setMotors(self.MAX_SPEED, -(0.25 * self.MAX_SPEED))

        # Turning Left
        elif self.ir1_detect == 1 and self.ir2_detect == 1 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(-(0.5 * self.MAX_SPEED), self.MAX_SPEED)

        # Turning Right
        elif self.ir1_detect == 0 and self.ir2_detect == 1 and self.ir3_detect == 1 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(0, self.MAX_SPEED)

        # Turning angular right 1
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 0 and self.ir6_detect == 1 and self.ir7_detect == 1 and self.ir8_detect == 0:
            self.setMotors(self.MAX_SPEED, 0)

        # Turning angular right 2
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 1 and self.ir6_detect == 1 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(self.MAX_SPEED, 0.5 * self.MAX_SPEED)

        # Turning angular left 1
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 1 and self.ir4_detect == 1 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(0.5 * self.MAX_SPEED, self.MAX_SPEED)

        # Turning angular left 2
        elif self.ir1_detect == 0 and self.ir2_detect == 1 and self.ir3_detect == 1 and self.ir4_detect == 1 and self.ir5_detect == 0 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(0.25 * self.MAX_SPEED, self.MAX_SPEED)

        # Acute angle turn towards right
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 0 and self.ir4_detect == 0 and self.ir5_detect == 1 and self.ir6_detect == 1 and self.ir7_detect == 1 and self.ir8_detect == 0:
            self.setMotors(self.MAX_SPEED, 0.25 * self.MAX_SPEED)

        # Acute angle turn towards left
        elif self.ir1_detect == 0 and self.ir2_detect == 0 and self.ir3_detect == 1 and self.ir4_detect == 1 and self.ir5_detect == 1 and self.ir6_detect == 0 and self.ir7_detect == 0 and self.ir8_detect == 0:
            self.setMotors(self.MAX_SPEED, 0.5 * self.MAX_SPEED)

        self.publishMotorCommands()

    def setMotors(self, left_speed, right_speed):
        self.left_mot = left_speed
        self.right_mot = right_speed

    def publishMotorCommands(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = (self.left_mot + self.right_mot) / 2.0
        cmd_vel_msg.angular.z = (self.right_mot - self.left_mot) / 0.3
        self.cmd_vel_pub.publish(cmd_vel_msg)


def main():
    rospy.init_node("line_follower")
    line_follower = LineFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
