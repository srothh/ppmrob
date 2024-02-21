#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSensor():

    # publish image data
    def publish(self, event=None):
        try:
            frameReader = self._drone.get_frame_read()
            frame = frameReader.frame
            msg = self._br.cv2_to_imgmsg(frame, encoding='rgb8')
            #rospy.loginfo(msg)
            self._publisher.publish(msg)
            self._counter += 1
        except Exception as e:
            rospy.loginfo(e)


    def __init__(self, drone):
        self._publisher = rospy.Publisher('/drone/camera', Image, queue_size=10)
        self._drone = drone
        self._br = CvBridge()
        self._counter = 0
        resp = drone.command('streamon')
        rospy.loginfo('ImageSensor initalized: %s' % resp)

