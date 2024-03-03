#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv.cv_node.msg import CoordinateList, PointPair
from geometry_msgs.msg import Point
import sys
import os

parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(parent_dir)
from PIL import Image
from sensor_msgs.msg import Image as Image_msg
from image_processing.detect_line import detect_lines
from image_processing.detect_victim import classify_image, yolo_detection
from image_processing.display_image import display_image, display_object_detection
from image_processing.process_image import img_processing
from util.util import build_coordinate_msg
import cv2

pub_victim = None
pub_lines = None


def callback(data, args):
    rospy.loginfo("Received image frame: %d %dx%d" % (data.height, data.height, data.width))
    br = CvBridge()
    # note: swich encoding to bgr8
    frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    # show image
    # cv2.imshow("Image window", frame)
    # cv2.waitKey(3)
    # save image
    # cv2.imwrite('lastframe.png', frame)
    detected, lines = img_processing(frame, args[0])
    # publish
    if pub_lines is not None and lines is not None:
        lines_msg = CoordinateList()
        lines_msg.point_pairs = []
        if lines is not None:
            lines_msg = build_coordinate_msg(lines, lines_msg)
        pub_lines.publish(lines_msg)
    if pub_victim is not None:
        victim_msg = CoordinateList()
        victim_msg.point_pairs = []
        if detected is not None:
            victim_msg = build_coordinate_msg(detected,victim_msg)
        pub_victim.publish(victim_msg)


def cv_node():
    global pub_lines, pub_victim
    # Initialize the ROS node
    rospy.init_node('cv_node', anonymous=True)
    # UNCOMMENT THIS TO TEST THE CLASSIFY_IMAGE FUNCTION (or0001.jpg needs to be in src directory)
    # DELETE THIS FOR TESTING WITH DRONE
    #    frame = cv2.imread('/catkin_ws/src/cv/src/or0001.jpg')
    #    window_name = 'contours'
    #    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    #    img_processing(frame, window_name)
    # STOP DELETE
    # Subscribe to the 'chatter' topic and register the callback function
    rospy.Subscriber('camera/forward', Image_msg, callback)
    pub_victim = rospy.publisher('cv/victim', CoordinateList)  # change message type
    pub_lines = rospy.publisher('cv/lines', CoordinateList)  # change message type
    print("Started CV NODE")
    # Spin to keep the script from exiting
    rospy.spin()


if __name__ == '__main__':
    cv_node()
