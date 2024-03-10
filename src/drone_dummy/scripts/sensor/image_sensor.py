#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import common.config.defaults
import os

class ImageSensor():


    # publish image data
    def publish(self, event=None):
        try:
            idx = self._counter % len(self._files)
            file = self._files[idx]
            frame = cv2.imread(file, cv2.IMREAD_COLOR)
            if frame.size == 0:
                raise Exception("file not found: %s" % file)
            msg = self._br.cv2_to_imgmsg(frame, encoding='rgb8')
            #rospy.loginfo(msg)
            self._publisher.publish(msg)
            self._counter += 1
        except Exception as e:
            rospy.loginfo(e)

    # return list off all png files from folder
    def get_files(self):
        files = []
        for file in os.listdir(self._datadir):
            if file.endswith(".png"):
                files.append(os.path.join(self._datadir, file))
        return files

    def __init__(self, datadir):
        self._publish_to_topic = common.config.defaults.drone_image_sensor_publish_topic_name
        self._publisher = rospy.Publisher(self._publish_to_topic, Image, queue_size=10)
        self._datadir = datadir
        self._br = CvBridge()
        self._counter = 0
        self._files = self.get_files()
        if len(self._files)==0:
            rospy.logerr("no image files found in %s" % self._datadir)
        
    

