#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from djitellopy import Tello

class ImageSensor():

    # publish image data
    def publish(self, event=None):
            if self._drone.get_current_state() and not self._drone.stream_on:
                self._drone.streamon()
            else:
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
        self._publisher = rospy.Publisher('camera/forward', Image, queue_size=10)
        self._drone = drone
        self._br = CvBridge()
        self._counter = 0
        rospy.loginfo('ImageSensor initalized')


if __name__ == '__main__':
    try:
        rospy.init_node('image')
        drone = Tello()
        drone.connect()
        sensor = ImageSensor(drone)
        rospy.Timer(rospy.Duration(1.0), sensor.publish)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
