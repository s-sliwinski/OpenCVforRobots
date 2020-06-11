#!/usr/bin/python

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class HumanRecognision:
    def __init__(self):
        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.convert_callback)
        self.img_pub = rospy.Publisher('human_detector_img', Image)
        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


    def convert_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        self.find_human(cv_image)

    def find_human(self, frame):
        frame = cv2.resize(frame, (640,480))
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        boxes, weights = self.hog.detectMultiScale(frame_gray)

        boxes = [[x,y,x+w,y+h] for (x,y,w,h) in boxes]

        for (xA,yA,xB,yB) in boxes:
            cv2.rectangle(frame, (xA,yA), (xB,yB), (0,255,0), 3)

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        except CvBridgeError as e:
            print(e)

        self.img_pub.publish(img_msg)

def main():
    hr = HumanRecognision()
    rospy.init_node('human_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down...")



if __name__ == "__main__":
    main()
