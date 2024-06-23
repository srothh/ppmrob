#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import common.config.defaults
import time

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


    def __init__(self, drone, topic=None):
        self._publish_to_topic = topic
        self._publisher = rospy.Publisher(self._publish_to_topic, Image, queue_size=10)
        self._drone = drone
        self._br = CvBridge()
        self._counter = 0
        resp = drone.command('streamon')
        time.sleep(0.1)
        resp = drone.command('downvision 1')
        rospy.loginfo('ImageSensor initalized: %s' % resp)

