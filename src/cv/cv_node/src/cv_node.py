#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Polygon
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
from util.util import build_coordinate_msg, build_polygon_msg
import cv2

pub_victim = None
pub_lines = None

"""
published topics:
cv/victim: Contains info on detected victims
cv/lines: Contains detected lines
Both publish a geometry_msgs/Polygon message, and every pair of Point32 in the message represents either
(start_point,end_point) for lines or (lower_left,upper_right) for the bounding box of the victim.
"""


def callback(data):
    rospy.loginfo("Received image frame: %d %dx%d" % (data.height, data.height, data.width))
    br = CvBridge()
    # note: swich encoding to bgr8
    frame = br.imgmsg_to_cv2(data, desired_encoding='bgr8')
    # show image
    # cv2.imshow("Image window", frame)
    # cv2.waitKey(3)
    # save image
    # cv2.imwrite('lastframe.png', frame)
    detected, lines = img_processing(frame)
    # publish
    if pub_lines is not None:
        lines_msg = Polygon()
        lines_msg.points = []
        if lines is not None:
            lines_msg = build_polygon_msg(lines, lines_msg)
        pub_lines.publish(lines_msg)
    if pub_victim is not None:
        victim_msg = Polygon()
        victim_msg.points = []
        if detected is not None:
            victim_msg = build_polygon_msg(detected, victim_msg)
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

    rospy.Subscriber('/drone/camera', Image_msg, callback)
    pub_victim = rospy.Publisher('/cv/victim', Polygon, queue_size=10)  # change message type
    pub_lines = rospy.Publisher('/cv/lines', Polygon, queue_size=10)  # change message type
    print("Started CV NODE")

    # Spin to keep the script from exiting
    rospy.spin()


if __name__ == '__main__':
    cv_node()