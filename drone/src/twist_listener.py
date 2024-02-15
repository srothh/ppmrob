#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
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
    cv2.imwrite('lastframe.png', frame)

def twist_node():
    # Initialize the ROS node
    rospy.init_node('imagelistener', anonymous=True)

    rospy.Subscriber('/drone/camera', Image, callback)

    # Spin to keep the script from exiting
    rospy.spin()

if __name__ == '__main__':
    twist_node()
