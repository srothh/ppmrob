#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
#from PIL import Image

class ImageRecorder:

    def __init__(self, topic, dir):
        self._counter = 0
        self._dir = dir
        # Create the directory
        if not os.path.exists(dir):
            os.makedirs(dir)
            rospy.loginfo("created dir: %s" % dir)
        self._br = CvBridge()
        self.record(topic)
  
    def record(self, topic):
        #rospy.init_node('image_recorder', anonymous=True)
        rospy.Subscriber(topic, Image, self.save)
        rospy.loginfo("Recording Images from %s to %s" % (topic, self._dir))

    
    def save(self, data):
        rospy.logdebug("Received image frame: %d %dx%d" % (data.height, data.height, data.width))
        # note: swich encoding to bgr8 
        frame = self._br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        #save image
        cv2.imwrite(self._dir + '/frame-{:04d}.png'.format(self._counter), frame)
        self._counter += 1

if __name__ == '__main__':
    recorder = ImageRecorder('/drone/camera', '.')
    # Spin to keep the script from exiting
    rospy.spin()
