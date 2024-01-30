#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(parent_dir)
from image_processing.detect_line import detect_lines
from image_processing.detect_victim import classify_image
import cv2
#from PIL import Image

def callback(data):
    rospy.loginfo("Received image frame: %d %dx%d" % (data.height, data.height, data.width))
    br = CvBridge()
    # note: swich encoding to bgr8
    frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    # show image
    #cv2.imshow("Image window", frame)
    #cv2.waitKey(3)
    #save image
    #cv2.imwrite('lastframe.png', frame)
    print(classify_image(frame))
    detect_lines(frame)
def cv_node():
    # Initialize the ROS node
    rospy.init_node('cv_node', anonymous=True)

    # Subscribe to the 'chatter' topic and register the callback function
    rospy.Subscriber('camera/forward', Image, callback)
    print("Started CV NODE")

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    cv_node()
