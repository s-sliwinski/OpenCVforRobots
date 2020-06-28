#!/usr/bin/python

from __future__ import print_function
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import numpy as np
import ast

class ChaseHuman:
    def __init__(self):
        self.box_pos_sub = rospy.Subscriber('/box_position', String, self.box_pos_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist)
        self.frame_size = self.get_frame_size()
        self.mid_x_size = self.frame_size[0]//2

    def box_pos_callback(self,data):
        # parsing string to
        box_pos = ast.literal_eval(data.data)
        self.calculate_vel(box_pos)

    def get_frame_size(self):
        rospy.wait_for_service('frame_size_srv')
        try:
            frame_size_srv_handle = rospy.ServiceProxy('frame_size_srv', Trigger)
            response = frame_size_srv_handle()
            return ast.literal_eval(response.message)
        except rospy.ServiceException as e:
            print("Service failed" + e)

    def calculate_vel(self,box_pos):

        cmd = Twist()

        if len(box_pos) == 0:
            print ("TARGET LOST")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            left_x_border, right_x_border = box_pos[0], box_pos[2]
            cmd.linear.x = 0.26
            if left_x_border > self.mid_x_size:
                cmd.angular.z = -0.3
            elif right_x_border < self.mid_x_size:
                cmd.angular.z = 0.3

        self.cmd_pub.publish(cmd)

def main():
    ch = ChaseHuman()
    rospy.init_node("chase_human", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down...")

if __name__=="__main__":
    main()
